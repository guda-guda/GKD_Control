#include "device/rc_controller.hpp"

#include "io.hpp"
#include "serial_interface.hpp"
#include "types.hpp"

namespace Device
{

    Rc_Controller::Rc_Controller(const std::string &serial_name) : serial_name(serial_name) {
    }

    void Rc_Controller::enable() {
        auto serial_interface = IO::io<SERIAL>[serial_name];
        if (serial_interface == nullptr) {
            LOG_ERR("RC_CONTROLLER Error: no serial named %s\n", serial_name.c_str());
            return;
        }
        serial_interface->register_callback<Types::ReceivePacket_RC_CTRL>(
            [&](const Types::ReceivePacket_RC_CTRL &rp) { unpack(rp); });
    }

    void Rc_Controller::unpack(const Types::ReceivePacket_RC_CTRL &pkg) {
        // LOG_INFO("rc controller ch0 %d\n", pkg.ch0);
        update_time();
    }
}  // namespace Device
