#include "AP_EscMonitor.h"

#include <AP_HAL/AP_HAL.h>

#if HAL_WITH_UAVCAN
#include "AP_EscMonitor_UAVCAN.h"
#endif

#include <AP_Vehicle/AP_Vehicle_Type.h>
#include <AP_Logger/AP_Logger.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_Notify/AP_Notify.h>

#include <AP_Param/AP_Param.h>

#include <AR_WPNav/AR_WPNav.h>

extern const AP_HAL::HAL& hal;

AP_EscMonitor *AP_EscMonitor::_singleton;

// Default constructor.
// Note that the Vector/Matrix constructors already implicitly zero
// their values.
//
AP_EscMonitor::AP_EscMonitor(esc_failsafe_handler_fn_t esc_failsafe_handler_fn) :
    _esc_failsafe_handler_fn(esc_failsafe_handler_fn)
{

    if (_singleton != nullptr) {
        AP_HAL::panic("AP_EscMonitor must be singleton");
    }
    _singleton = this;
}

// init - instantiate the battery monitors
void
AP_EscMonitor::init()
{
    // check init has not been called before
    if (_num_instances != 0) {
        return;
    }
}

void AP_EscMonitor::check_failsafe()
{

    if (!_has_triggered_failsafe)
    {
        #ifdef HAVE_AP_BLHELI_SUPPORT
            AP_BLHeli *blheli = AP_BLHeli::get_singleton();
            if (blheli) {
                if (blheli->check_esc_temperature_reached_threshold())
                {
                    _has_triggered_failsafe = true;
                    _esc_failsafe_handler_fn();
                }
                
            }
        #endif
    }



}

namespace AP {

AP_EscMonitor &esc()
{
    return *AP_EscMonitor::get_singleton();
}

};
