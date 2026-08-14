#pragma once

// LocalJoystick: direct non-blocking reader for a Linux joystick device
// (/dev/input/jsX), for real-robot runs that read the gamepad locally instead
// of subscribing to the DDS RemoteControllerState topic.
//
// The normalized values follow the real-robot remote convention (matches the
// physical remote used on the Booster K1, and the RemoteControllerState
// published by booster_mujoco's fillRemoteControllerState):
//   lx = axis[0]/max, ly = +axis[1]/max, rx = axis[2]/max, ry = +axis[5]/max
//   buttons: a=1 b=2 x=0 y=3 lb=4 rb=5 back=8 start=9 (device button index)
//   lt = axis[3] > 0, rt = axis[4] > 0
//   d-pad from axis[6] (left/right) and axis[7] (up/down), negative = left/up
// so the state can be fed straight into Gamepad::set_raw without further
// scaling.  Deadzone/smoothing are applied downstream by Gamepad, matching the
// DDS path.

#include <cerrno>
#include <cstdint>
#include <fcntl.h>
#include <linux/joystick.h>
#include <string>
#include <sys/ioctl.h>
#include <unistd.h>

namespace booster_deploy
{

class LocalJoystick
{
public:
    explicit LocalJoystick(std::string device_path = "/dev/input/js0", int bits = 16)
        : device_path_(std::move(device_path)), max_value_(1 << (bits - 1)) {}

    bool open()
    {
        fd_ = ::open(device_path_.c_str(), O_RDONLY | O_NONBLOCK);
        if (fd_ < 0)
        {
            return false;
        }
        char name[128] = {0};
        if (::ioctl(fd_, JSIOCGNAME(sizeof(name)), name) >= 0)
        {
            device_name_ = name;
        }
        return true;
    }

    void close()
    {
        if (fd_ >= 0)
        {
            ::close(fd_);
            fd_ = -1;
        }
    }

    ~LocalJoystick() { close(); }

    bool isOpen() const { return fd_ >= 0; }

    const std::string &deviceName() const { return device_name_; }

    // Drain all pending events.  Returns true if the device is healthy (either
    // events were consumed or none are pending), false if it was unplugged or
    // a read error occurred -- the caller should then stop polling so the last
    // known state is kept.
    bool poll()
    {
        if (fd_ < 0)
        {
            return false;
        }
        for (;;)
        {
            ssize_t n = ::read(fd_, &event_, sizeof(event_));
            if (n == static_cast<ssize_t>(sizeof(event_)))
            {
                if (event_.type & JS_EVENT_BUTTON)
                {
                    button_[event_.number & 0x7f] = event_.value;
                }
                else if (event_.type & JS_EVENT_AXIS)
                {
                    axis_[event_.number & 0x7f] = event_.value;
                }
                continue;
            }
            if (n == -1 && errno == EAGAIN)
            {
                return true;  // drained, device alive
            }
            return false;  // EOF / error / unplugged
        }
    }

    // ---- normalized axes (RemoteControllerState convention) ----
    float lx() const { return axis_[0] / max_value_; }
    float ly() const { return axis_[1] / max_value_; }
    float rx() const { return axis_[2] / max_value_; }
    float ry() const { return axis_[5] / max_value_; }

    // ---- buttons (device button index) ----
    bool a() const { return button_[1] != 0; }
    bool b() const { return button_[2] != 0; }
    bool x() const { return button_[0] != 0; }
    bool y() const { return button_[3] != 0; }
    bool lb() const { return button_[4] != 0; }
    bool rb() const { return button_[5] != 0; }
    bool back() const { return button_[8] != 0; }
    bool start() const { return button_[9] != 0; }

    // ---- triggers (analog axes 3/4, pressed beyond half travel) ----
    bool lt() const { return axis_[3] > 0; }
    bool rt() const { return axis_[4] > 0; }

    // ---- d-pad from axes 6/7 (negative = left/up) ----
    bool hatUp() const { return axis_[7] < 0; }
    bool hatDown() const { return axis_[7] > 0; }
    bool hatLeft() const { return axis_[6] < 0; }
    bool hatRight() const { return axis_[6] > 0; }

    // raw access for debugging / test tools
    int rawAxis(unsigned i) const { return i < 10 ? axis_[i] : 0; }
    int rawButton(unsigned i) const { return i < 20 ? button_[i] : 0; }

private:
    std::string device_path_;
    std::string device_name_;
    int fd_ = -1;
    float max_value_;
    struct js_event event_ {};
    int axis_[10] = {0};
    int button_[20] = {0};
};

} // namespace booster_deploy
