#include "iot/thing.h"
#include "board.h"
#include "audio_codec.h"

#include <esp_log.h>

#define TAG "Camera"

namespace iot {
    class Camera : public Thing{
    private:
        /* data */
        bool _isOpen = false;
    public:
        Camera() : Thing("Camera","摄像头"), _isOpen(false) {
            // 定义设备的属性
            properties_.AddBooleanProperty("isopen", "摄像头是否打开", [this]() -> bool {
                return _isOpen;
            });

              // 定义设备可以被远程执行的指令
            methods_.AddMethod("CameraOn", "打开摄像头", ParameterList(), [this](const ParameterList& parameters) {
                _isOpen = true;
            });

            methods_.AddMethod("CameraOff", "关闭摄像头", ParameterList(), [this](const ParameterList& parameters) {
                _isOpen = false;
            });
        };
    };
    
} // namespace iot

DECLARE_THING(Camera);