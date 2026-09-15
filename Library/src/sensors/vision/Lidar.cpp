/*    
    This file is a part of Stonefish.

    Stonefish is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    Stonefish is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/

//
//  Lidar.cpp
//  Stonefish
//
//  Created by Pedro Pereira on 30/06/2026.
//

#include "sensors/vision/Lidar.h"

#include "core/GraphicalSimulationApp.h"
#include "core/SimulationManager.h"
#include "graphics/OpenGLPipeline.h"
#include "graphics/OpenGLContent.h"
#include "graphics/OpenGLDepthCamera.h"
#include "entities/forcefields/Ocean.h"
#include <glm/glm.hpp>


namespace sf
{

Lidar::Lidar(std::string uniqueName, Scalar verticalFOV, uint channels, Scalar horizontalFOV, uint steps,
             Scalar rangeMin, Scalar rangeMax, uint frequency, bool filter) : Camera(uniqueName, steps, channels, horizontalFOV, frequency)
{
    fovV = verticalFOV;
    fovH = horizontalFOV;
    range.x = rangeMin < Scalar(0.01) ? 0.01f : (GLfloat)rangeMin;
    range.y = rangeMax > Scalar(0.01) ? (GLfloat)rangeMax : 1.f;
    newDataCallback = NULL;
    dataCounter = 0;
    marchWater = filter;

    const float targetPrecision = 0.3f;
    // Binary-search steps to reach targetPrecision over the full range: log2(range/precision).
    maxBinarySearchSteps = (int)ceil(std::log2((range.y - range.x) / targetPrecision));

    resY = channels;   // vertical channels (elevation rows)
    resX = steps;      // horizontal steps (azimuth columns)

    imageData = new GLfloat[resX*resY];
    memset(imageData, 0, resX*resY*sizeof(GLfloat));
    rangeData = new GLfloat[resX*resY];
    memset(rangeData, 0, resX*resY*sizeof(GLfloat));
}

Lidar::~Lidar()
{
    if(imageData != NULL) delete [] imageData;
    if(rangeData != NULL) delete [] rangeData;
    cameras.clear();   // views already freed + nulled by RemoveFromGraphics
    rayDirs.clear();
}

void Lidar::SetNoise(Scalar distance)
{
    noise_dist = distance;
}

void* Lidar::getImageDataPointer(unsigned int index)
{
    if(cameras.size() > index)
        return &imageData[cameras[index].dataOffset];
    else
        return NULL;
}

float* Lidar::getRangeDataPointer()
{
    return rangeData;
}

const glm::vec3* Lidar::getRayDirections() const
{
    return rayDirs.empty() ? nullptr : rayDirs.data();
}

glm::vec2 Lidar::getRangeLimits() const
{
    return range;
}

Scalar Lidar::getVerticalFOV() const
{
    return fovV;
}

Scalar Lidar::getHorizontalFOV() const
{
    return fovH;
}

VisionSensorType Lidar::getVisionSensorType() const
{
    return VisionSensorType::LIDAR;
}

OpenGLView* Lidar::getOpenGLView() const
{
    if(cameras.size() > 0)
        return cameras[0].cam;
    else
        return nullptr;
}

void Lidar::InitGraphics(bool& seesParticles) //Ignore seesParticles
{
    //--- 1. Decide how many perspective tiles are needed to cover fovH ---
    if(fovH <= Scalar(LIDAR_MAX_SINGLE_FOV))
    {
        LidarCamData cd;
        cd.cam = NULL;
        cd.fovH = (GLfloat)fovH;
        cd.width = resX;
        cd.dataOffset = 0;
        cameras.push_back(cd);
    }
    else
    {
        int nCam = (int)ceil(fovH/Scalar(LIDAR_MAX_SINGLE_FOV));
        Scalar fovH1 = fovH/Scalar(nCam);
        int resmod = resX % nCam;
        if(resmod == 0)
        {
            for(int i=0; i<nCam; ++i)
            {
                LidarCamData cd;
                cd.cam = NULL;
                cd.fovH = (GLfloat)fovH1;
                cd.width = resX/nCam;
                cd.dataOffset = 0;
                cameras.push_back(cd);
            }
        }
        else
        {
            int resX1 = resX/nCam;
            int resXc = resX1 + resmod;
            Scalar fovHc = fovH/(Scalar(1) + (nCam-1)*Scalar(resX1)/Scalar(resXc));
            fovH1 = fovHc * Scalar(resX1)/Scalar(resXc);

            LidarCamData cd;
            cd.cam = NULL;
            cd.dataOffset = 0;
            cd.fovH = (GLfloat)fovHc;
            cd.width = resXc;
            cameras.push_back(cd);

            cd.fovH = (GLfloat)fovH1;
            cd.width = resX1;
            for(int i=0; i<nCam-1; ++i) cameras.push_back(cd);
        }
    }

    //--- 2. Assign each tile its center azimuth (sensor-local, REP-103: CCW from +X) ---
    // Same accumulation convention as the per-tile rotation in UpdateTransform,
    // so camera setup and the direction LUT can never disagree.
    {
        Scalar offset = fovH/Scalar(360)*M_PI;   // half of fovH in radians
        Scalar accFov(0);
        for(size_t i=0; i<cameras.size(); ++i)
        {
            Scalar halfFov = cameras[i].fovH/Scalar(360)*M_PI;
            cameras[i].centerAzimuth = (GLfloat)(offset - accFov - halfFov);
            accFov += Scalar(2)*halfFov;
        }
    }

    //--- 3. Create the depth cameras (one per tile) ---
    GLint accResX = 0;
    for(size_t i=0; i<cameras.size(); ++i)
    {
        cameras[i].cam = new OpenGLDepthCamera(
                        glm::vec3(0,0,0), glm::vec3(1.f,0,0), glm::vec3(0,0,1.f),
                        accResX, 0, cameras[i].width, resY,
                        cameras[i].fovH, range.x, range.y,
                        true,            // continuousUpdate
                        true,            // useRanges -> Euclidean metric range per pixel
                        (GLfloat)fovV);  // verticalFOVDeg -> manual projection, even beam spacing
        cameras[i].cam->setCamera(this, (unsigned int)i);
        cameras[i].dataOffset = accResX*resY;
        accResX += cameras[i].width;
    }

    //--- 4. Precompute per-cell unit ray directions in the sensor frame ---
    BuildRayDirections();

    //--- 5. Place cameras in the world and register ---
    UpdateTransform();
    for(size_t i=0; i<cameras.size(); ++i)
    {
        cameras[i].cam->UpdateTransform();
        cameras[i].cam->Update();
        ((GraphicalSimulationApp*)SimulationApp::getApp())->getGLPipeline()->getContent()->AddView(cameras[i].cam);
    }
}

void Lidar::BuildRayDirections()
{
    // Build the exact direction of every (column, row) sample in the sensor-local
    // REP-103 frame (X forward, Y left, Z up). Computed from the actual perspective
    // geometry of each tile, so multiplying by the Euclidean range gives the correct
    // 3D point with no tan() blow-up, no azimuth/elevation coupling error, no frame twist.
    rayDirs.assign((size_t)resX * (size_t)resY, glm::vec3(0.f));

    const float tanV = tanf((float)fovV * (float)M_PI / 360.f);   // tan(fovV/2)

    size_t colBase = 0;
    for(size_t t=0; t<cameras.size(); ++t)
    {
        const float psi  = cameras[t].centerAzimuth;             // tile look azimuth
        const float tanH = tanf(cameras[t].fovH * (float)M_PI / 360.f); // tan(tileFovH/2)
        const int   w    = cameras[t].width;

        // Sensor-frame basis of this tile's camera:
        //   forward = (cos psi, sin psi, 0)   (camera looks along this; maps to eye -Z)
        //   up      = (0, 0, 1)               (sensor Z; vertical image axis -> elevation)
        //   right   = cross(forward, up) = (sin psi, -cos psi, 0)  (horizontal image axis)
        const glm::vec3 fwd  (cosf(psi), sinf(psi), 0.f);
        const glm::vec3 up   (0.f, 0.f, 1.f);
        const glm::vec3 right(sinf(psi), -cosf(psi), 0.f);

        for(int hl=0; hl<w; ++hl)
        {
            const size_t h = colBase + (size_t)hl;
            const float ndc_x = 2.f*((float)hl + 0.5f)/(float)w - 1.f;
            const float ex = ndc_x * tanH;   // eye-space x / |z|

            for(unsigned int v=0; v<resY; ++v)
            {
                // Row 0 = bottom of the rendered image (OpenGL readback order) = lowest beam.
                // If the cloud comes out vertically mirrored, negate ndc_y.
                const float ndc_y = 2.f*((float)v + 0.5f)/(float)resY - 1.f;
                const float ey = ndc_y * tanV;   // eye-space y / |z|

                // Eye ray (ex, ey, -1) expressed in the sensor frame:
                //   d = ex*right + ey*up + 1*forward
                glm::vec3 d = ex*right + ey*up + fwd;
                rayDirs[(size_t)v*(size_t)resX + h] = glm::normalize(d);
            }
        }
        colBase += (size_t)w;
    }
}

void Lidar::InternalUpdate(Scalar dt)
{
    for(size_t i=0; i<cameras.size(); ++i)
        cameras[i].cam->Update();
}

void Lidar::UpdateTransform()
{
    Transform tf = getSensorFrame();
    Vector3 eyePosition = tf.getOrigin();
    Vector3 spinAxis = tf.getBasis().getColumn(2);   // sensor Z (world) -> rotation/up axis
    Vector3 baseDir  = tf.getBasis().getColumn(0);   // sensor X (world) -> azimuth 0 look
    Vector3 cameraUp = spinAxis;                     // up == spin axis (valid: look is horizontal)

    for(size_t i=0; i<cameras.size(); ++i)
    {
        // Single rotation of the base look direction about the spin axis by this
        // tile's center azimuth. (centerAzimuth was set once in InitGraphics.)
        Vector3 dir = baseDir.rotate(spinAxis, Scalar(cameras[i].centerAzimuth));
        SetupCamera(i, eyePosition, dir, cameraUp);
    }
}

void Lidar::SetupCamera(const Vector3& eye, const Vector3& dir, const Vector3& up)
{
}

void Lidar::SetupCamera(size_t index, const Vector3& eye, const Vector3& dir, const Vector3& up)
{
    glm::vec3 eye_ = glm::vec3((GLfloat)eye.x(), (GLfloat)eye.y(), (GLfloat)eye.z());
    glm::vec3 dir_ = glm::vec3((GLfloat)dir.x(), (GLfloat)dir.y(), (GLfloat)dir.z());
    glm::vec3 up_  = glm::vec3((GLfloat)up.x(),  (GLfloat)up.y(),  (GLfloat)up.z());
    cameras[index].cam->SetupCamera(eye_, dir_, up_);
}

void Lidar::InstallNewDataHandler(std::function<void(Lidar*)> callback)
{
    newDataCallback = callback;
}

void Lidar::NewDataReady(void* data, unsigned int index)
{
    if(index >= cameras.size())
        return;

    memcpy(getImageDataPointer(index), data, cameras[index].width * resY * sizeof(GLfloat));
    dataCounter += index;
    int lastIndex = (int)cameras.size()-1;
    int nSum = lastIndex*(lastIndex+1)/2;

    if(dataCounter != nSum)   // not all tiles received yet
        return;

    dataCounter = 0;

    //Reassemble tile strips into one (resX x resY) range grid: rangeData[h + v*resX]
    for(size_t i=0; i<cameras.size(); ++i)
    {
        size_t xoffset = cameras[i].dataOffset/resY;
        for(size_t h=0; h<resY; ++h)
        {
            memcpy(&rangeData[xoffset + h*resX],
                   &imageData[cameras[i].dataOffset + h*cameras[i].width],
                   sizeof(GLfloat)*cameras[i].width);
        }
    }

    Ocean* ocean = SimulationApp::getApp()->getSimulationManager()->getOcean();
    if(ocean != nullptr)
    {
        if(marchWater)
        {
            uint64_t seed = (uint64_t)(SimulationApp::getApp()->getSimulationManager()->getSimulationTime() * 1000.0);
            ApplyWaterInteraction(seed);
        }
        else
        {
            CropUnderwater();   // flat / no-march: just drop points below the surface
        }
    }

    if(newDataCallback != NULL)
        newDataCallback(this);
}

void Lidar::ApplyWaterInteraction(uint64_t seed)
{
    // ---- Constants hoisted once (were re-fetched inside the loops) ----
    Ocean* ocean = SimulationApp::getApp()->getSimulationManager()->getOcean();
    Transform tf = getSensorFrame();
    const Matrix3 basis = tf.getBasis();
    const Vector3 originVec = tf.getOrigin();
    const glm::vec3 sensorPos((float)originVec.x(), (float)originVec.y(), (float)originVec.z());

    const size_t totalBeams = (size_t)resX * (size_t)resY;
    const float invCell = 1.0f / cellXY;

    // Reuse scratch; clear() keeps capacity, so no per-scan allocation.
    activeSearches_.clear();
    boundaryQueries_.clear();
    heightCache_.clear();
    if(boundaryQueries_.capacity() < totalBeams * 2)
        boundaryQueries_.reserve(totalBeams * 2);

    // Snap a world position to its height-cache cell center, registering the cell
    // in heightCache_ (value filled later by the batched query).
    auto cellCenter = [&](const glm::vec3& p, int& gX, int& gY) -> glm::vec3
    {
        gX = (int)std::floor(p.x * invCell);
        gY = (int)std::floor(p.y * invCell);
        return glm::vec3(((float)gX + 0.5f) * cellXY, ((float)gY + 0.5f) * cellXY, 0.0f);
    };

    // ========================================================
    // PHASE 1: geometric prune + boundary batch to find candidates
    // ========================================================
    // For each in-range beam we cache its world direction ONCE (in the search
    // struct) and queue its near/far endpoints for a single batched depth query.
    // A beam that can never reach the water (origin above the surface, ray heading
    // away from it) is pruned here with no FFT sample at all.

    // We don't know the exact wave surface Z cheaply, but the flat mean level is 0
    // in world Z for the ocean plane; a beam above it heading further up cannot cross.
    const bool sensorAboveMean = sensorPos.z < 0.0f; // world Z<0 = above water (depth>0 is submerged)

    for(size_t i = 0; i < totalBeams; ++i)
    {
        const float rMax = rangeData[i];
        if(rMax < range.x || rMax > range.y)
            continue;

        // World-space ray direction, computed exactly once for this beam.
        const glm::vec3 rSensor = rayDirs[i];
        const Vector3 dW = basis * Vector3(rSensor.x, rSensor.y, -rSensor.z);
        const glm::vec3 rWorld((float)dW.x(), (float)dW.y(), (float)dW.z());

        // Geometric prune: sensor above the mean surface and ray heading upward
        // (away from water in world Z) can never cross. Skip without any query.
        if(sensorAboveMean && rWorld.z <= 0.0f)
            continue;

        boundaryQueries_.push_back(sensorPos + rWorld * (float)range.x); // near
        boundaryQueries_.push_back(sensorPos + rWorld * rMax);           // far
        activeSearches_.push_back({ i, rWorld, (float)range.x, rMax });  // rWorld cached here
    }

    if(activeSearches_.empty())
        return;

    // One batched depth query for all near/far endpoints.
    std::vector<float> boundaryDepths = ocean->GetDepthMap(boundaryQueries_);

    // Classify each candidate; keep only those with a guaranteed crossing.
    // (Compact activeSearches_ in place: survivors move to the front.)
    size_t kept = 0;
    for(size_t k = 0; k < activeSearches_.size(); ++k)
    {
        const float startDepth = boundaryDepths[k * 2];
        const float endDepth   = boundaryDepths[k * 2 + 1];

        if(startDepth > 0.0f)         // near end already submerged -> flooded beam
        {
            rangeData[activeSearches_[k].beamIdx] = 0.0f;
            continue;
        }
        if(endDepth <= 0.0f)          // both ends in air -> no (end) crossing; keep hit
            continue;                 // NOTE: misses mid-ray crest occlusion by design

        // Guaranteed air->water crossing between minR (=range.x) and maxR (=rMax).
        activeSearches_[kept++] = activeSearches_[k]; // maxR already = rMax from Phase 1
    }
    activeSearches_.resize(kept);
    if(activeSearches_.empty())
        return;

    // ========================================================
    // PHASE 2: batched binary-search interval halving
    // ========================================================
    // Each step: compute every active beam's midpoint ONCE, batch the unique
    // height-cache cells, then split each interval using the cached height.
    for(int step = 0; step < maxBinarySearchSteps; ++step)
    {
        // 1. Queue unique cells for all midpoints (one entry per world cell).
        midQueries_.clear();
        heightCache_.clear();
        for(const LidarSearch& s : activeSearches_)
        {
            const float midR = (s.minR + s.maxR) * 0.5f;
            const glm::vec3 midPos = sensorPos + s.rWorld * midR;
            int gX, gY;
            glm::vec3 center = cellCenter(midPos, gX, gY);
            auto res = heightCache_.emplace(GridKey{gX, gY}, 0.0f);
            if(res.second)                       // newly inserted -> needs a query
                midQueries_.push_back(center);
        }

        // 2. One batched height query for the unique cells.
        if(!midQueries_.empty())
        {
            std::vector<float> depths = ocean->GetDepthMap(midQueries_);
            // Write each queried cell's surface height back into the cache.
            // The query points ARE the cell centers, so re-derive the cell from
            // each to stay consistent with midQueries_ ordering.
            for(size_t q = 0; q < midQueries_.size(); ++q)
            {
                int gX = (int)std::floor(midQueries_[q].x * invCell);
                int gY = (int)std::floor(midQueries_[q].y * invCell);
                heightCache_[GridKey{gX, gY}] = -depths[q]; // surface Z at Z=0 sample = -depth
            }
        }

        // 3. Split each interval on its (cached) midpoint depth.
        for(LidarSearch& s : activeSearches_)
        {
            const float midR = (s.minR + s.maxR) * 0.5f;
            const glm::vec3 midPos = sensorPos + s.rWorld * midR;
            int gX = (int)std::floor(midPos.x * invCell);
            int gY = (int)std::floor(midPos.y * invCell);
            const float waveHeight = heightCache_[GridKey{gX, gY}];
            const float midDepth = midPos.z - waveHeight;

            if(midDepth > 0.0f) s.maxR = midR; // midpoint underwater -> crossing is nearer
            else                s.minR = midR; // midpoint in air     -> crossing is farther
        }
    }

    // ========================================================
    // PHASE 3: final assignment through the stochastic water model
    // ========================================================
    for(const LidarSearch& s : activeSearches_)
    {
        const float crossingRange = (s.minR + s.maxR) * 0.5f;
        rangeData[s.beamIdx] = WaterReturn(s.beamIdx, crossingRange, seed);
    }
}

void Lidar::CropUnderwater()
{
    Transform tf = getSensorFrame();
    const Matrix3 basis = tf.getBasis();
    const Vector3 originVec = tf.getOrigin();
    const glm::vec3 sensorPos((float)originVec.x(), (float)originVec.y(), (float)originVec.z());
    const size_t totalBeams = (size_t)resX * (size_t)resY;

    for(size_t i = 0; i < totalBeams; ++i)
    {
        const float r = rangeData[i];
        if(r < range.x || r > range.y)
            continue;

        const glm::vec3 rSensor = rayDirs[i];
        const Vector3 dW = basis * Vector3(rSensor.x, rSensor.y, -rSensor.z);
        const float hitZ = sensorPos.z + (float)dW.z() * r;   // world Z of the hit point

        if(hitZ > 0.0f)              // below the waterline (z>0 = underwater)
            rangeData[i] = 0.0f;     // drop
    }
}

float Lidar::WaterReturn(size_t beamIndex, float crossingRange, uint64_t seed)
{
    // Fast stateless hash -> [0,1). Reproducible and thread-safe (no shared state).
    auto hash = [](size_t x, uint64_t seed) {
        x = ((x >> 16) ^ x) * 0x45d9f3b + (size_t)seed;
        x = ((x >> 16) ^ x) * 0x45d9f3b;
        x = (x >> 16) ^ x;
        return (float)(x % 10000) / 10000.f;
    };

    const float r1 = hash(beamIndex, seed);
    const float r2 = hash(beamIndex + 9999, seed);

    // 1. Specular dropout (most common over water) -> lost beam.
    if(r1 < dropoutProb)
        return 0.0f;

    // 2. Straggler (multipath / sub-surface scatter) -> erroneous deeper return.
    if(r1 < dropoutProb + stragglerProb)
        return crossingRange + (r2 * stragglerSpread);

    // 3. Surface return with Gaussian range jitter (Box-Muller).
    const float randNormal = sqrtf(-2.0f * logf(fmaxf(r1, 0.0001f))) * cosf(2.0f * (float)M_PI * r2);
    return fmaxf(0.01f, crossingRange + randNormal * noiseSigma);
}

std::vector<Renderable> Lidar::Render()
{
    std::vector<Renderable> items = Sensor::Render();
    if(isRenderable())
    {
        Renderable item;
        item.model = glMatrixFromTransform(getSensorFrame());
        item.type = RenderableType::SENSOR_LINES;
        item.data = std::make_shared<std::vector<glm::vec3>>();
        auto points = item.getDataAsPoints();
        
        unsigned int div = (unsigned int)ceil(fovH/5.0);
        GLfloat iconSize = 0.5f;
        GLfloat cosFovV2 = cosf(fovV/360.f*M_PI);
        GLfloat sinFovV2 = sinf(fovV/360.f*M_PI);
        GLfloat r = iconSize/cosFovV2;
        GLfloat thetaDiv = fovH/180.f * M_PI/(GLfloat)div;
        GLfloat offset = -fovH/360.f * M_PI;
        GLfloat y = sinFovV2 * r;
        
        for(unsigned int i=0; i<div; ++i)
        {
            GLfloat theta1 = i*thetaDiv + offset;
            GLfloat theta2 = theta1 + thetaDiv;
            GLfloat d1 = cosf(theta1) * r;
            GLfloat d2 = cosf(theta2) * r;
            GLfloat z1 = cosFovV2 * d1;
            GLfloat z2 = cosFovV2 * d2;
            GLfloat x1 = sinf(theta1) * r;
            GLfloat x2 = sinf(theta2) * r;
            
            points->push_back(glm::vec3(x1,y,z1));
            points->push_back(glm::vec3(x2,y,z2));
            points->push_back(glm::vec3(x1,-y,z1));
            points->push_back(glm::vec3(x2,-y,z2));
            
            if(i == 0) //End 1
            {
                points->push_back(glm::vec3(x1,y,z1));
                points->push_back(glm::vec3(x1,-y,z1));
                points->push_back(glm::vec3(x1,y,z1));
                points->push_back(glm::vec3(0,0,0));
                points->push_back(glm::vec3(x1,-y,z1));
                points->push_back(glm::vec3(0,0,0));
            }
            else if(i == div-1) //End 2
            {
                points->push_back(glm::vec3(x2,y,z2));
                points->push_back(glm::vec3(x2,-y,z2));
                points->push_back(glm::vec3(x2,y,z2));
                points->push_back(glm::vec3(0,0,0));
                points->push_back(glm::vec3(x2,-y,z2));
                points->push_back(glm::vec3(0,0,0));
            }
        }
        
        items.push_back(item);
    }
    return items;
}

void Lidar::RemoveFromGraphics()   // in Lidar.h/.cpp, override
{
    // TODO: OpenGLContent remove view not implemented yet
    // if(SimulationApp::getApp()->hasGraphics())
    // {
    //     auto* content = ((GraphicalSimulationApp*)SimulationApp::getApp())->getGLPipeline()->getContent();
    //     for(size_t i = 0; i < cameras.size(); ++i)
    //     {
    //         if(cameras[i].cam != nullptr)
    //         {
    //             content->RemoveView(cameras[i].cam);  // unlink + delete on render thread, GL current
    //             cameras[i].cam = nullptr;              // so ~Lidar won't double-free
    //         }
    //     }
    // }
}

}
