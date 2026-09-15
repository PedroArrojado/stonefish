#include "sensors/scalar/RangeFinder.h"
#include "core/GraphicalSimulationApp.h"
#include "core/SimulationManager.h"
#include "graphics/OpenGLPipeline.h"
#include "graphics/OpenGLContent.h"

namespace sf
{

Rangefinder::Rangefinder(std::string uniqueName, Scalar rangeMin, Scalar rangeMax, 
                         uint frequency, Scalar fovDeg) 
    : Camera(uniqueName, 1, 1, fovDeg, frequency), depthCam(nullptr), rangeData(0.0f)
{
    fov = (GLfloat)fovDeg;
    rangeLimits.x = rangeMin < Scalar(0.01) ? 0.01f : (GLfloat)rangeMin;
    rangeLimits.y = rangeMax > Scalar(0.01) ? (GLfloat)rangeMax : 1.0f;
    newDataCallback = nullptr;
}

Rangefinder::~Rangefinder()
{
}

void Rangefinder::SetupCamera(const Vector3& eye, const Vector3& dir, const Vector3& up)
{
    if (depthCam != nullptr)
    {
        glm::vec3 eye_((GLfloat)eye.x(), (GLfloat)eye.y(), (GLfloat)eye.z());
        glm::vec3 dir_((GLfloat)dir.x(), (GLfloat)dir.y(), (GLfloat)dir.z());
        glm::vec3 up_ ((GLfloat)up.x(),  (GLfloat)up.y(),  (GLfloat)up.z());
        depthCam->SetupCamera(eye_, dir_, up_);
    }
}

void* Rangefinder::getImageDataPointer(unsigned int index)
{
    // Return pointer to internal single range float
    return &rangeData;
}

VisionSensorType Rangefinder::getVisionSensorType() const
{
    // Return custom or existing optical sensor type
    return VisionSensorType::DEPTH_CAMERA; 
}

OpenGLView* Rangefinder::getOpenGLView() const
{
    return depthCam;
}

void Rangefinder::InitGraphics(bool& seesParticles) //Ignore seesParticles
{
    // Configure a 1x1 depth camera view with Euclidean range calculation
    depthCam = new OpenGLDepthCamera(
        glm::vec3(0.0f), glm::vec3(1.0f, 0.0f, 0.0f), glm::vec3(0.0f, 0.0f, 1.0f),
        0, 0, 1, 1,                     // 1x1 pixel resolution
        fov, rangeLimits.x, rangeLimits.y,
        true,                           // continuousUpdate
        true,                           // useRanges = true (Euclidean distance metric)
        fov                             // verticalFOVDeg = fov (Square pixel)
    );

    depthCam->setCamera(this, 0);

    UpdateTransform();
    depthCam->UpdateTransform();
    depthCam->Update();

    ((GraphicalSimulationApp*)SimulationApp::getApp())->getGLPipeline()->getContent()->AddView(depthCam);
}

void Rangefinder::UpdateTransform()
{
    if (depthCam == nullptr) return;

    Transform tf = getSensorFrame();
    Vector3 eye = tf.getOrigin();
    Vector3 dir = tf.getBasis().getColumn(0); // Forward (+X in sensor frame)
    Vector3 up  = tf.getBasis().getColumn(2); // Up (+Z in sensor frame)

    glm::vec3 eye_((GLfloat)eye.x(), (GLfloat)eye.y(), (GLfloat)eye.z());
    glm::vec3 dir_((GLfloat)dir.x(), (GLfloat)dir.y(), (GLfloat)dir.z());
    glm::vec3 up_ ((GLfloat)up.x(),  (GLfloat)up.y(),  (GLfloat)up.z());

    depthCam->SetupCamera(eye_, dir_, up_);
}

void Rangefinder::InternalUpdate(Scalar dt)
{
    if (depthCam != nullptr)
        depthCam->Update();
}

void Rangefinder::SetNoise(Scalar std)
{
    return; // Not implemented yet
}


void Rangefinder::NewDataReady(void* data, unsigned int index)
{
    // Extract single float from 1x1 depth buffer
    rangeData = ((GLfloat*)data)[0];

    // Out-of-bounds check
    if (rangeData < rangeLimits.x || rangeData > rangeLimits.y)
        rangeData = rangeLimits.y;

    if (newDataCallback != nullptr)
        newDataCallback(this);

    // Should it interact with water?
}

float Rangefinder::getRange() const
{
    return rangeData;
}

float* Rangefinder::getRangeDataPointer()
{
    return &rangeData;
}

void Rangefinder::InstallNewDataHandler(std::function<void(Rangefinder*)> callback)
{
    newDataCallback = callback;
}

void Rangefinder::RemoveFromGraphics()
{
    // OpenGLContent remove view not implemented yet
    // if (SimulationApp::getApp()->hasGraphics() && depthCam != nullptr)
    // {
    //     auto* content = ((GraphicalSimulationApp*)SimulationApp::getApp())->getGLPipeline()->getContent();
    //     content->RemoveView(depthCam);
    //     depthCam = nullptr;
    // }
}

std::vector<Renderable> Rangefinder::Render()
{
    std::vector<Renderable> items = Sensor::Render();
    if (isRenderable())
    {
        Renderable item;
        item.model = glMatrixFromTransform(getSensorFrame());
        item.type = RenderableType::SENSOR_LINES;
        auto points = item.getDataAsPoints();

        // Draw a single laser ray forward line (+X axis) for visual debug
        points->push_back(glm::vec3(0.0f, 0.0f, 0.0f));
        points->push_back(glm::vec3(rangeLimits.y, 0.0f, 0.0f));

        items.push_back(item);
    }
    return items;
}

} // namespace sf