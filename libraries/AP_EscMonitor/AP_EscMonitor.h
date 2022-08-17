#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS_MAVLink.h>

class AP_EscMonitor
{

public:

    // battery failsafes must be defined in levels of severity so that vehicles wont fall backwards
    enum BatteryFailsafe {
        BatteryFailsafe_None = 0,
        BatteryFailsafe_Low,
        BatteryFailsafe_Critical
    };

    FUNCTOR_TYPEDEF(esc_failsafe_handler_fn_t, void);

    AP_EscMonitor(esc_failsafe_handler_fn_t battery_failsafe_handler_fn);

    /* Do not allow copies */
    AP_EscMonitor(const AP_EscMonitor &other) = delete;
    AP_EscMonitor &operator=(const AP_EscMonitor&) = delete;

    static AP_EscMonitor *get_singleton() {
        return _singleton;
    }

    // Return the number of battery monitor instances
    uint8_t num_instances(void) const { return _num_instances; }

    // detect and initialise any available battery monitors
    void init();

    // If using the BLHeli ESCs, calls BLHeli's check_esc_temperature_reached_threshold() function to check for failsafe trigger.
    void check_failsafe();

    /// returns true if a battery failsafe has ever been triggered
    bool has_failsafed(void) const { return _has_triggered_failsafe; };

private:
    static AP_EscMonitor *_singleton;

    uint8_t     _num_instances;                                     /// number of monitors

    esc_failsafe_handler_fn_t _esc_failsafe_handler_fn;
    bool        _has_triggered_failsafe;  // true after a battery failsafe has been triggered for the first time

};

namespace AP {
    AP_EscMonitor &esc();
};

