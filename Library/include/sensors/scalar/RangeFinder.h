#ifndef RANGEFINDER_H
#define RANGEFINDER_H

#include "sensors/vision/Camera.h"
#include "graphics/OpenGLDepthCamera.h"
#include <glm/glm.hpp>
#include <functional>

namespace sf
{

class Rangefinder : public Camera
{
public:
    Rangefinder(std::string uniqueName, Scalar rangeMin, Scalar rangeMax, 
                uint frequency, Scalar fovDeg = Scalar(0.1));
    virtual ~Rangefinder();

    // VisionSensor overrides
    virtual VisionSensorType getVisionSensorType() const override;
    virtual OpenGLView* getOpenGLView() const override;

    // Graphics lifecycle
    virtual void InitGraphics(bool& seesParticles) override;
    virtual void RemoveFromGraphics() override;
    virtual void UpdateTransform() override;
    virtual void InternalUpdate(Scalar dt) override;

    // Data retrieval
    float getRange() const;
    float* getRangeDataPointer();
    void InstallNewDataHandler(std::function<void(Rangefinder*)> callback);
    void NewDataReady(void* data, unsigned int index);

    float getMaxRange() const { return rangeLimits.y; }
    float getMinRange() const { return rangeLimits.x; }
    float getFOV() const { return static_cast<float>(fov); }
    
    void SetupCamera(const Vector3& eye, const Vector3& dir, const Vector3& up);
    virtual void* getImageDataPointer(unsigned int index);

    void SetNoise(Scalar std);


    virtual std::vector<Renderable> Render() override;

private:
    OpenGLDepthCamera* depthCam;
    GLfloat rangeData;           // Single scalar range value [m]
    glm::vec2 rangeLimits;       // Min and max range
    GLfloat fov;                 // Pencil-thin FOV (e.g., 0.1 deg)

    std::function<void(Rangefinder*)> newDataCallback;
};

} // namespace sf

#endif // RANGEFINDER_H