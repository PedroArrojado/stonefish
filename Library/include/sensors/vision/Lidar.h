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
//  Lidar.h
//  Stonefish
//
//  Created by Pedro Pereira on 30/06/2026.
//

#ifndef __Stonefish_Lidar__
#define __Stonefish_Lidar__

#include <functional>
#include <vector>
#include <unordered_map>
#include "StonefishCommon.h"
#include "sensors/vision/Camera.h"
#include "graphics/OpenGLDataStructs.h"


namespace sf
{
    //! Maximum horizontal FOV a single perspective sub-camera may cover [deg].
    //! Wider sensors are tiled across several cameras (perspective breaks down
    //! as the FOV approaches 180 deg).
    #define LIDAR_MAX_SINGLE_FOV 90

    class OpenGLDepthCamera;

    //! Discrete world-XY grid cell key for the wave-height cache.
    struct GridKey
    {
        int x;
        int y;
        bool operator==(const GridKey& o) const { return x == o.x && y == o.y; }
    };

    //! Hash so GridKey can be used in an unordered_map.
    struct GridKeyHash
    {
        size_t operator()(const GridKey& k) const
        {
            return (std::hash<int>()(k.x) ^ (std::hash<int>()(k.y) << 1));
        }
    };

    //! A structure holding data for a single sub-camera tile.
    struct LidarCamData
    {
        OpenGLDepthCamera* cam;
        int width;              //!< number of horizontal samples (columns) in this tile
        GLfloat fovH;           //!< horizontal FOV of this tile [deg]
        GLfloat centerAzimuth;  //!< tile look azimuth in sensor frame [rad], CCW from +X
        size_t dataOffset;      //!< offset into imageData where this tile's strip begins
    };

    //! A structure tracking one beam's bracketed water-crossing search.
    //! rWorld is cached once (Phase 1) so the binary-search loop never recomputes it.
    struct LidarSearch
    {
        size_t beamIdx;
        glm::vec3 rWorld;   //!< world-space unit ray direction (constant for the scan)
        float minR;         //!< bracket: known-air range
        float maxR;         //!< bracket: known-water range
    };

    //! A class representing a rotating 3D LIDAR (or a 2D scanner when channels == 1).
    /*!
        Built as a set of perspective depth-camera tiles that cover the horizontal FOV,
        each rendering a (width x channels) grid of Euclidean ranges. Output is a point
        cloud in the sensor-local REP-103 frame (X forward, Y left, Z up); azimuth is
        measured CCW about +Z, elevation about the horizontal plane.
     */
    class Lidar : public Camera
    {
    public:
        //! A constructor.
        /*!
         \param uniqueName a name for the sensor
         \param verticalFOV the vertical field of view [deg]
         \param channels the number of vertical channels (beams) -> resY
         \param horizontalFOV the horizontal field of view [deg]
         \param steps the number of horizontal samples per revolution -> resX
         \param rangeMin the minimum measured range [m]
         \param rangeMax the maximum measured range [m]
         \param frequency the sampling frequency [Hz] (-1 for rendering rate)
         */
        Lidar(std::string uniqueName, Scalar verticalFOV, uint channels, Scalar horizontalFOV, uint steps,
              Scalar rangeMin, Scalar rangeMax, uint frequency, bool filter = true);

        //! A destructor.
        ~Lidar();

        //! A method performing internal sensor state update.
        void InternalUpdate(Scalar dt) override;

        //! A method updating the transform of the sub-cameras.
        void UpdateTransform() override;

        //! A method initializing the OpenGL resources (sub-cameras + direction table).
        void InitGraphics(bool& seesParticles) override;

        //! A method to set the range noise standard deviation [m].
        void SetNoise(Scalar distance);

        //! A method handling new data delivered from one sub-camera.
        void NewDataReady(void* data, unsigned int index);

        //! A method to add water-interface noise to a beam that crossed the surface.
        /*!
        \param beamIndex the beam id (hash seed)
        \param crossingRange distance from the sensor to the water surface [m]
        \param seed per-scan seed for hash reproducibility
        \return the (possibly dropped/jittered/straggled) range [m]; 0 = dropped
        */
        float WaterReturn(size_t beamIndex, float crossingRange, uint64_t seed);

        //! A method used to set up one of the sub-cameras (by index).
        void SetupCamera(size_t index, const Vector3& eye, const Vector3& dir, const Vector3& up);

        //! Unused single-camera override (kept for interface compatibility).
        void SetupCamera(const Vector3& eye, const Vector3& dir, const Vector3& up);

        //! A method installing a callback fired when a full scan is ready.
        void InstallNewDataHandler(std::function<void(Lidar*)> callback);

        //! A method returning a pointer to a sub-camera's raw image data.
        void* getImageDataPointer(unsigned int index = 0);

        //! A method returning a pointer to the reassembled (resX x resY) range grid [m].
        float* getRangeDataPointer();

        //! A method returning the per-cell unit ray directions (sensor frame, REP-103).
        const glm::vec3* getRayDirections() const;

        //! A method returning the min/max range limits [m].
        glm::vec2 getRangeLimits() const;

        //! A method returning the vertical field of view [deg].
        Scalar getVerticalFOV() const;

        //! A method returning the horizontal field of view [deg].
        Scalar getHorizontalFOV() const;

        //! A method returning the type of the vision sensor.
        VisionSensorType getVisionSensorType() const;

        //! A method returning the OpenGL view (first sub-camera).
        OpenGLView* getOpenGLView() const;

        //! A method returning items that should be rendered for the sensor icon.
        std::vector<Renderable> Render();

        void RemoveFromGraphics();

    private:
        //! Precompute the per-cell unit ray directions in the sensor frame.
        void BuildRayDirections();

        //! Apply the water surface interaction (crop + crossing search + noise) to rangeData.
        void ApplyWaterInteraction(uint64_t seed);

        void CropUnderwater();

        GLfloat fovV;            //!< vertical field of view [deg]
        GLfloat fovH;            //!< horizontal field of view [deg]
        glm::vec2 range;         //!< min/max range [m]
        Scalar noise_dist;       //!< range noise std-dev [m]

        std::vector<LidarCamData> cameras;
        std::vector<glm::vec3> rayDirs;   //!< per-cell unit directions, sensor frame [resX*resY]

        GLfloat* imageData;      //!< scratch buffer for per-tile strips
        GLfloat* rangeData;      //!< reassembled (resX x resY) range grid [m]
        int dataCounter;         //!< counts received tiles per scan

        // Controls
        bool marchWater = true;
        int oceanState = 1;      //!< 0 = flat ocean, >0 = waves active

        // Tunable knobs
        float cellXY = 0.5f;     //!< world height-cache resolution [m]
        int maxBinarySearchSteps;

        // Stochastic water-return parameters
        float dropoutProb = 0.95f;    //!< probability a water hit drops out (specular miss)
        float noiseSigma = 0.05f;     //!< surface-return range jitter std-dev [m]
        float stragglerProb = 0.05f;  //!< probability of a scattered error return
        float stragglerSpread = 3.0f; //!< range spread for stragglers [m]

        // Per-scan reusable scratch (members: no per-scan heap allocation, no
        // cross-instance sharing). Cleared (not freed) between scans, so capacity
        // is retained.
        std::vector<LidarSearch> activeSearches_;
        std::vector<glm::vec3> boundaryQueries_;
        std::vector<glm::vec3> midQueries_;
        std::unordered_map<GridKey, float, GridKeyHash> heightCache_;

        std::function<void(Lidar*)> newDataCallback;
    };
}

#endif
