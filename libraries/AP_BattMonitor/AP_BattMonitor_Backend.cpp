/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include "AP_BattMonitor.h"
#include "AP_BattMonitor_Backend.h"
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Stats/AP_Stats.h>

#include <iostream>
#include <vector>

/*
  base class constructor.
  This incorporates initialisation as well.
*/
AP_BattMonitor_Backend::AP_BattMonitor_Backend(AP_BattMonitor &mon, AP_BattMonitor::BattMonitor_State &mon_state,
                                               AP_BattMonitor_Params &params) :
        _mon(mon),
        _state(mon_state),
        _params(params)
{
}

/// capacity_remaining_pct - returns the % battery capacity remaining (0 ~ 100)
uint8_t AP_BattMonitor_Backend::capacity_remaining_pct() const
{
    float mah_remaining = _params._pack_capacity - _state.consumed_mah;
    if ( _params._pack_capacity > 10 ) { // a very very small battery
        return MIN(MAX((100 * (mah_remaining) / _params._pack_capacity), 0), UINT8_MAX);
    } else {
        return 0;
    }
}

// update battery resistance estimate
// faster rates of change of the current and voltage readings cause faster updates to the resistance estimate
// the battery resistance is calculated by comparing the latest current and voltage readings to a low-pass filtered current and voltage
// high current steps are integrated into the resistance estimate by varying the time constant of the resistance filter
void AP_BattMonitor_Backend::update_resistance_estimate()
{
    // return immediately if no current
    if (!has_current() || !is_positive(_state.current_amps)) {
        return;
    }

    // update maximum current seen since startup and protect against divide by zero
    _current_max_amps = MAX(_current_max_amps, _state.current_amps);
    float current_delta = _state.current_amps - _current_filt_amps;
    if (is_zero(current_delta)) {
        return;
    }

    // update reference voltage and current
    if (_state.voltage > _resistance_voltage_ref) {
        _resistance_voltage_ref = _state.voltage;
        _resistance_current_ref = _state.current_amps;
    }

    // calculate time since last update
    uint32_t now = AP_HAL::millis();
    float loop_interval = (now - _resistance_timer_ms) / 1000.0f;
    _resistance_timer_ms = now;

    // estimate short-term resistance
    float filt_alpha = constrain_float(loop_interval/(loop_interval + AP_BATT_MONITOR_RES_EST_TC_1), 0.0f, 0.5f);
    float resistance_alpha = MIN(1, AP_BATT_MONITOR_RES_EST_TC_2*fabsf((_state.current_amps-_current_filt_amps)/_current_max_amps));
    float resistance_estimate = -(_state.voltage-_voltage_filt)/current_delta;
    if (is_positive(resistance_estimate)) {
        _state.resistance = _state.resistance*(1-resistance_alpha) + resistance_estimate*resistance_alpha;
    }

    // calculate maximum resistance
    if ((_resistance_voltage_ref > _state.voltage) && (_state.current_amps > _resistance_current_ref)) {
        float resistance_max = (_resistance_voltage_ref - _state.voltage) / (_state.current_amps - _resistance_current_ref);
        _state.resistance = MIN(_state.resistance, resistance_max);
    }

    // update the filtered voltage and currents
    _voltage_filt = _voltage_filt*(1-filt_alpha) + _state.voltage*filt_alpha;
    _current_filt_amps = _current_filt_amps*(1-filt_alpha) + _state.current_amps*filt_alpha;

    // update estimated voltage without sag
    _state.voltage_resting_estimate = _state.voltage + _state.current_amps * _state.resistance;
}

float AP_BattMonitor_Backend::voltage_resting_estimate() const
{
    // resting voltage should always be greater than or equal to the raw voltage
    return MAX(_state.voltage, _state.voltage_resting_estimate);
}

AP_BattMonitor::BatteryFailsafe AP_BattMonitor_Backend::update_failsafes(void)
{
    const uint32_t now = AP_HAL::millis();

    bool low_voltage, low_capacity, critical_voltage, critical_capacity, custom_low_capacity;
    check_failsafe_types(low_voltage, low_capacity, critical_voltage, critical_capacity, custom_low_capacity);

    if (critical_voltage) {
        // this is the first time our voltage has dropped below minimum so start timer
        if (_state.critical_voltage_start_ms == 0) {
            _state.critical_voltage_start_ms = now;
        } else if (_params._low_voltage_timeout > 0 &&
                   now - _state.critical_voltage_start_ms > uint32_t(_params._low_voltage_timeout)*1000U) {
            // return AP_BattMonitor::BatteryFailsafe_Critical;
        }
    } else {
        // acceptable voltage so reset timer
        _state.critical_voltage_start_ms = 0;
    }

    // if (critical_capacity) {
    //     return AP_BattMonitor::BatteryFailsafe_Critical;
    // }

    if (low_voltage) {
        // this is the first time our voltage has dropped below minimum so start timer
        if (_state.low_voltage_start_ms == 0) {
            _state.low_voltage_start_ms = now;
        } else if (_params._low_voltage_timeout > 0 &&
                   now - _state.low_voltage_start_ms > uint32_t(_params._low_voltage_timeout)*1000U) {
            return AP_BattMonitor::BatteryFailsafe_Low;
        }
    } else {
        // acceptable voltage so reset timer
        _state.low_voltage_start_ms = 0;
    }

    if (custom_low_capacity){
        std::cout<<"LOW CAP ????"<<std::endl;
        return AP_BattMonitor::BatteryFailsafe_Critical;
    }

    // if (low_capacity) {
    //     return AP_BattMonitor::BatteryFailsafe_Low;
    // }

    // if we've gotten this far then battery is ok
    return AP_BattMonitor::BatteryFailsafe_None;
}


double AP_BattMonitor_Backend::_get_distance_to_home(void) const
{

    const AP_AHRS &ahrs = AP::ahrs();

    Location home_loc =  ahrs.get_home();

    Location current_loc;
    ahrs.get_position(current_loc);

    double distance = current_loc.get_distance(home_loc);
    return distance;
}

static bool update_check(size_t buflen, char *buffer, bool failed, const char *message)
{
    if (failed) {
        strncpy(buffer, message, buflen);
        return false;
    }
    return true;
}

bool AP_BattMonitor_Backend::arming_checks(char * buffer, size_t buflen) const
{
    bool low_voltage, low_capacity, critical_voltage, critical_capacity, custom_low_capacity;
    check_failsafe_types(low_voltage, low_capacity, critical_voltage, critical_capacity, custom_low_capacity);

    bool below_arming_voltage = is_positive(_params._arming_minimum_voltage) &&
                                (_state.voltage < _params._arming_minimum_voltage);
    bool below_arming_capacity = (_params._arming_minimum_capacity > 0) &&
                                 ((_params._pack_capacity - _state.consumed_mah) < _params._arming_minimum_capacity);
    bool fs_capacity_inversion = is_positive(_params._critical_capacity) &&
                                 is_positive(_params._low_capacity) &&
                                 (_params._low_capacity < _params._critical_capacity);
    bool fs_voltage_inversion = is_positive(_params._critical_voltage) &&
                                is_positive(_params._low_voltage) &&
                                (_params._low_voltage < _params._critical_voltage);

    bool result = update_check(buflen, buffer, low_voltage,  "low voltage failsafe");
    result = result && update_check(buflen, buffer, low_capacity, "low capacity failsafe");
    result = result && update_check(buflen, buffer, critical_voltage, "critical voltage failsafe");
    result = result && update_check(buflen, buffer, critical_capacity, "critical capacity failsafe");
    result = result && update_check(buflen, buffer, below_arming_voltage, "below minimum arming voltage");
    result = result && update_check(buflen, buffer, below_arming_capacity, "below minimum arming capacity");
    result = result && update_check(buflen, buffer, fs_capacity_inversion, "capacity failsafe critical > low");
    result = result && update_check(buflen, buffer, fs_voltage_inversion, "voltage failsafe critical > low");
    result = result && update_check(buflen, buffer, custom_low_capacity, "custom low capacity failsafe");

    return result;
}

void AP_BattMonitor_Backend::check_failsafe_types(bool &low_voltage, bool &low_capacity, bool &critical_voltage, bool &critical_capacity, bool &custom_low_capacity) const
{
    // use voltage or sag compensated voltage
    float voltage_used;
    switch (_params.failsafe_voltage_source()) {
        case AP_BattMonitor_Params::BattMonitor_LowVoltageSource_Raw:
        default:
            voltage_used = _state.voltage;
            break;
        case AP_BattMonitor_Params::BattMonitor_LowVoltageSource_SagCompensated:
            voltage_used = voltage_resting_estimate();
            break;
    }

    // check critical battery levels
    if ((voltage_used > 0) && (_params._critical_voltage > 0) && (voltage_used < _params._critical_voltage)) {
        critical_voltage = true;
    } else {
        critical_voltage = false;
    }

    // check capacity failsafe if current monitoring is enabled
    if (has_current() && (_params._critical_capacity > 0) &&
        ((_params._pack_capacity - _state.consumed_mah) < _params._critical_capacity)) {
        critical_capacity = true;
    } else {
        critical_capacity = false;
    }

    if ((voltage_used > 0) && (_params._low_voltage > 0) && (voltage_used < _params._low_voltage)) {
        std::cout<<"Low volt?"<<std::endl;
        low_voltage = true;
    } else {
        low_voltage = false;
    }

    // check capacity if current monitoring is enabled
    if (has_current() && (_params._low_capacity > 0) &&
        ((_params._pack_capacity - _state.consumed_mah) < _params._low_capacity)) {
        low_capacity = true;
    } else {
        low_capacity = false;
    }

    check_custom_failsafe(custom_low_capacity);

}

void AP_BattMonitor_Backend::check_custom_failsafe(bool &custom_failsafe) const
{

    custom_failsafe = false;    // set failsafe to false so that it will be set if the function returns early

    if (!param_values_set()){
        set_param_values();
        return;
    }

    std::cout<<"Checking custom failsafe..."<<std::endl;

    AP_Stats *stats = AP::stats();
    uint32_t _flight_time = stats->get_flight_time_s();

    if (_flight_time < 8) return;

    const float _resting_voltage = voltage_resting_estimate();

    float _adjusted_capacity_percent = -0.3568410913 * pow(_resting_voltage,4) + 35.0222463246 * pow(_resting_voltage,3) - 1290.1979687034 * pow(_resting_voltage,2) + 21157.7578769615 * _resting_voltage - 130310.4773939850;

    std::cout<<"Adjusted cap percent: " << _adjusted_capacity_percent<<std::endl;


    if ((int)_initial_percent_remaining == 0) _initial_percent_remaining = _adjusted_capacity_percent;

    float _horizontal_velocity = ((int)_rtl_speed != 0) ? _rtl_speed : _wpnav_speed;
    double _horizontal_distance = _get_distance_to_home();

    const AP_AHRS &ahrs = AP::ahrs();
    float _current_altitude = ahrs.get_home().alt;
    ahrs.get_relative_position_D_home(_current_altitude);
    _current_altitude = _current_altitude * -1; // altitude reported as negative; swap to positive;

    std::cout<<"Current alt: " <<_current_altitude<<std::endl;

    float _vertical_distance2 = _land_alt_low / 100;
    float _vertical_velocity2 = _land_speed / 100;

    if (_current_altitude < _vertical_distance2) _vertical_distance2 = _current_altitude;

    float _vertical_distance1 = _current_altitude - _vertical_distance2;

    if (_vertical_distance1 < 0) _vertical_distance1 = 0;

    float _vertical_velocity1  = ((int)_land_speed_high != 0) ? _land_speed_high : _wpnav_speed_dn;

    float _time_to_rtl = ((_horizontal_distance / _horizontal_velocity) + (_vertical_distance1 / _vertical_velocity1) + (_vertical_distance2 / _vertical_velocity2));



    float _capacity_rate = (_initial_percent_remaining - _adjusted_capacity_percent) / _flight_time;

    std::cout<<"Init percent: " << _initial_percent_remaining<<std::endl;
    std::cout<<"Flgiht time: "<< _flight_time<<std::endl;
    std::cout<<"Cap rate: " <<_capacity_rate<<std::endl;
    if (_capacity_rate <=0) return;

    float _time_until_twenty_percent_capacity = (_adjusted_capacity_percent - 20) / _capacity_rate;

    std::cout<<"Time to RTL: " << _time_to_rtl<<std::endl;
    std::cout<<"Time until 20%: " << _time_until_twenty_percent_capacity <<std::endl;
    
    if (_time_to_rtl >= _time_until_twenty_percent_capacity){
        std::cout<<"Custom battery failsafe triggered!"<<std::endl;
        custom_failsafe = true;
    }

    return;
}

bool AP_BattMonitor_Backend::param_values_set() const
{
    return _rtl_speed_set && _wpnav_speed_set && _land_alt_low_set && _land_speed_set && _land_speed_high_set && _wpnav_speed_dn_set;
}

void AP_BattMonitor_Backend::set_param_values() const
{
    enum ap_var_type p_type;
    AP_Param *vp;

    std::vector<std::string> parameter_names = {"WPNAV_SPEED", "RTL_SPEED", "LAND_ALT_LOW", "LAND_SPEED_HIGH", "WPNAV_SPEED_DN", "LAND_SPEED"};
    std::vector<float*> parameter_values = {&_wpnav_speed, &_rtl_speed, &_land_alt_low, &_land_speed_high, &_wpnav_speed_dn, &_land_speed};
    std::vector<bool*> parameter_values_set = {&_wpnav_speed_set, &_rtl_speed_set, &_land_alt_low_set, &_land_speed_high_set, &_wpnav_speed_dn_set, &_land_speed_set};

    for (uint i = 0; i < parameter_names.size(); i++)
    {
        // WPNAV_SPEED
        char param_name[AP_MAX_NAME_SIZE+1];
        memset(param_name, 0, sizeof param_name);

        strncpy(param_name, parameter_names[i].c_str(), parameter_names[i].length());
        // param_name = "WPNAV_SPEED";
        // param_name[AP_MAX_NAME_SIZE] = 0;
        vp = AP_Param::find(param_name, &p_type);
        if (vp != NULL) {
            *parameter_values[i] = vp->cast_to_float(p_type);
            *parameter_values_set[i] = true;
        }

        memset(param_name, 0, sizeof param_name);
    }

    parameter_names.clear();
    parameter_values.clear();
    parameter_values_set.clear();
}

/*
  default implementation for reset_remaining(). This sets consumed_wh
  and consumed_mah based on the given percentage. Use percentage=100
  for a full battery
*/
bool AP_BattMonitor_Backend::reset_remaining(float percentage)
{
    percentage = constrain_float(percentage, 0, 100);
    const float used_proportion = (100 - percentage) * 0.01;
    _state.consumed_mah = used_proportion * _params._pack_capacity;
    // without knowing the history we can't do consumed_wh
    // accurately. Best estimate is based on current voltage. This
    // will be good when resetting the battery to a value close to
    // full charge
    _state.consumed_wh = _state.consumed_mah * 1000 * _state.voltage;

    // reset failsafe state for this backend
    _state.failsafe = update_failsafes();

    return true;
}
