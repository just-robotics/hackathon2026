/**
 * @file /kobuki_driver/include/kobuki_driver/modules/acceleration_limiter.hpp
 *
 * @brief Simple module for the velocity smoothing.
 *
 * License: BSD
 *   https://raw.githubusercontent.com/kobuki-base/kobuki_core/license/LICENSE
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef KOBUKI_CORE_ACCELERATION_LIMITER_HPP_
#define KOBUKI_CORE_ACCELERATION_LIMITER_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <algorithm>
#include <cmath>
#include <vector>
#include <iomanip>
#include <sstream>
#include <iostream>
#include <stdint.h>
#include <ecl/time.hpp>

#include "../parameters.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace kobuki {

/*****************************************************************************
** Interfaces
*****************************************************************************/

/**
 * @brief An acceleration limiter for the kobuki.
 *
 * This class will check incoming velocity commands and limit them if
 * the change since the last incoming command is great.
 *
 * Limits are supplied via init() from kobuki::Parameters.
 * init(enable) uses default Parameters limits; the full
 * form sets them explicitly.
 *
 * Speed is clamped first, then acceleration/deceleration.
 * Acceleration and deceleration limits follow the sign of the current
 * velocity. A command that crosses zero brakes to zero first, then
 * accelerates into the new direction with the remaining time.
 */
class AccelerationLimiter {
public:
  AccelerationLimiter() :
    is_enabled(true),
    last_timestamp(ecl::TimeStamp()),
    last_vx(0.0),
    last_wz(0.0)
  {}
  void init(bool enable_acceleration_limiter)
  {
    Parameters defaults;
    init(enable_acceleration_limiter,
         defaults.linear_acceleration_limit,
         defaults.angular_acceleration_limit,
         defaults.linear_deceleration_limit,
         defaults.angular_deceleration_limit,
         defaults.linear_speed_limit,
         defaults.angular_speed_limit);
  }
  void init(bool enable_acceleration_limiter,
            double linear_acceleration_max_,
            double angular_acceleration_max_,
            double linear_deceleration_max_,
            double angular_deceleration_max_,
            double linear_speed_max_,
            double angular_speed_max_)
  {
    is_enabled = enable_acceleration_limiter;
    linear_acceleration_max  = std::abs(linear_acceleration_max_);
    linear_deceleration_max  = std::abs(linear_deceleration_max_);
    angular_acceleration_max = std::abs(angular_acceleration_max_);
    angular_deceleration_max = std::abs(angular_deceleration_max_);
    linear_speed_max = linear_speed_max_;
    angular_speed_max = angular_speed_max_;
  }

  bool isEnabled() const { return is_enabled; }

  /**
   * @brief Limits the input velocity commands if gatekeeper is enabled.
   *
   * What is the limit?
   *
   * @param command : translation and angular velocity components in a 2-dim vector.
   */
  std::vector<double> limit(const std::vector<double> &command) { return limit(command[0], command[1]); }

  std::vector<double> limit(const double &vx, const double &wz)
  {
    std::vector<double> ret_val;
    double vx_limited = std::max(-linear_speed_max, std::min(vx, linear_speed_max));
    double wz_limited = std::max(-angular_speed_max, std::min(wz, angular_speed_max));

    if( is_enabled ) {
      //get current time
      ecl::TimeStamp curr_timestamp;
      //get time difference
      ecl::TimeStamp duration = curr_timestamp - last_timestamp;
      const double dt = duration;

      command_vx = limitAxis(last_vx, vx_limited, dt,
                             linear_acceleration_max, linear_deceleration_max);
      last_vx = command_vx;

      command_wz = limitAxis(last_wz, wz_limited, dt,
                             angular_acceleration_max, angular_deceleration_max);
      last_wz = command_wz;

      last_timestamp = curr_timestamp;

      ret_val.push_back(command_vx);
      ret_val.push_back(command_wz);
    } else {
      last_vx = vx_limited;
      last_wz = wz_limited;
      ret_val.push_back(vx_limited);
      ret_val.push_back(wz_limited);
    }
    return ret_val;
  }

private:
  /**
   * @brief Limit one velocity axis over dt.
   *
   * Both limits are positive magnitudes: acceleration_max applies while
   * |v| grows, deceleration_max while |v| shrinks, in either direction.
   * Opposite signs split the step at zero.
   */
  static double limitAxis(const double v0, const double v1, const double dt,
                          const double acceleration_max, const double deceleration_max)
  {
    if( dt <= 0.0 ) {
      return v0;
    }

    if( (v0 > 0.0 && v1 < 0.0) || (v0 < 0.0 && v1 > 0.0) ) {
      const double t_stop = std::abs(v0) / deceleration_max;
      if( t_stop >= dt ) {
        return v0 - std::copysign(deceleration_max * dt, v0);
      }
      const double v = std::copysign(acceleration_max * (dt - t_stop), v1);
      return (v1 > 0.0) ? std::min(v, v1) : std::max(v, v1);
    }

    const double side = (v0 != 0.0) ? v0 : v1;
    if( side == 0.0 ) {
      return 0.0;
    }

    const double up   = (side > 0.0) ? acceleration_max : deceleration_max;
    const double down = (side > 0.0) ? deceleration_max : acceleration_max;
    return std::max(v0 - down * dt, std::min(v1, v0 + up * dt));
  }

  bool is_enabled;
  ecl::TimeStamp last_timestamp;

  double last_vx, last_wz; // In [m/s] and [rad/s]
  double command_vx, command_wz; // In [m/s] and [rad/s]
  double linear_acceleration_max, linear_deceleration_max; // Magnitudes in [m/s^2]
  double angular_acceleration_max, angular_deceleration_max; // Magnitudes in [rad/s^2]
  double linear_speed_max, angular_speed_max; // In [m/s] and [rad/s]
};

} // namespace kobuki

#endif /* KOBUKI_ACCELERATION_LIMITER__HPP_ */
