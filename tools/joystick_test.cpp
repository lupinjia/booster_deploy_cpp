// joystick_test: dump raw and normalized values from a local joystick device.
//
// Usage: ./joystick_test [device_path]
//   device_path defaults to /dev/input/js0.
//
// Prints, once per poll (~20ms):
//   - raw axis values (device range, e.g. -32768..32767)
//   - normalized axes lx/ly/rx/ry in [-1, 1] (RemoteControllerState convention)
//   - button states (a/b/x/y/lb/rb/back/start)
//   - trigger states (lt/rt) and d-pad (hat) states
// Press Ctrl-C to exit.  Useful to verify that the deployed joystick mapping
// (see local_joystick.hpp) reads the expected values before a real-robot run.

#include <chrono>
#include <cstdio>
#include <string>
#include <thread>

#include "local_joystick.hpp"

int main(int argc, char **argv)
{
    std::string device = argc >= 2 ? argv[1] : "/dev/input/js0";
    booster_deploy::LocalJoystick js(device);
    if (!js.open())
    {
        std::perror("open");
        std::fprintf(stderr, "cannot open joystick device %s\n", device.c_str());
        return 1;
    }
    std::printf("device: %s  (%s)\n", device.c_str(), js.deviceName().c_str());
    std::printf("mapping (real-robot remote convention):\n");
    std::printf("  lx=axis0/max  ly=+axis1/max  rx=axis2/max  ry=+axis5/max\n");
    std::printf("  buttons: a=1 b=2 x=0 y=3 lb=4 rb=5 back=8 start=9\n");
    std::printf("  lt=axis3>0  rt=axis4>0  hat: axis6<0=L/>0=R, axis7<0=U/>0=D\n");
    std::printf("  press buttons / move sticks to see values change; Ctrl-C to quit\n\n");

    while (true)
    {
        if (!js.poll())
        {
            std::fprintf(stderr, "\ndevice read error (unplugged?) -- exiting\n");
            return 2;
        }
        std::printf(
            "raw  axis0=%6d axis1=%6d axis2=%6d axis3=%6d axis4=%6d axis5=%6d axis6=%6d axis7=%6d\n",
            js.rawAxis(0), js.rawAxis(1), js.rawAxis(2), js.rawAxis(3),
            js.rawAxis(4), js.rawAxis(5), js.rawAxis(6), js.rawAxis(7));
        std::printf(
            "norm lx=%+6.3f ly=%+6.3f rx=%+6.3f ry=%+6.3f\n",
            js.lx(), js.ly(), js.rx(), js.ry());
        std::printf(
            "btn  a=%d b=%d x=%d y=%d lb=%d rb=%d back=%d start=%d   lt=%d rt=%d\n",
            js.a(), js.b(), js.x(), js.y(), js.lb(), js.rb(), js.back(), js.start(),
            js.lt(), js.rt());
        std::printf(
            "hat  U=%d D=%d L=%d R=%d\n\n",
            js.hatUp(), js.hatDown(), js.hatLeft(), js.hatRight());
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
}
