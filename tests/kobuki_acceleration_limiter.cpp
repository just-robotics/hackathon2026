// Standalone regression check; no USB/device access.
#include <algorithm>
#include <cmath>
#include <vector>
#include <iomanip>
#include <sstream>
#include <stdint.h>
#include <ecl/time.hpp>
#include <kobuki_core/parameters.hpp>
// Access the pure one-axis calculation for deterministic time-step checks.
#define private public
#include <kobuki_core/modules/acceleration_limiter.hpp>
#undef private
#include <cassert>
#include <iostream>

static void near(double actual, double expected) {
  assert(std::abs(actual - expected) < 1e-9);
}
int main() {
  using L = kobuki::AccelerationLimiter;
  near(L::limitAxis(0., .4, .1, .3, .7), .03);
  near(L::limitAxis(0., -.4, .1, .3, .7), -.03);
  near(L::limitAxis(.3, .4, .1, .3, .7), .33);
  near(L::limitAxis(-.3, -.4, .1, .3, .7), -.33);
  near(L::limitAxis(.3, 0., .1, .3, .7), .23);
  near(L::limitAxis(-.3, 0., .1, .3, .7), -.23);
  near(L::limitAxis(.2, -.4, .1, .3, .7), .13);
  near(L::limitAxis(-.2, .4, .1, .3, .7), -.13);
  near(L::limitAxis(.2, -.4, .5, .3, .7), -.3 * (.5 - .2 / .7));
  near(L::limitAxis(-.2, .4, .5, .3, .7), .3 * (.5 - .2 / .7));
  near(L::limitAxis(.2, -.4, 0., .3, .7), .2);
  near(L::limitAxis(.2, -.4, -.1, .3, .7), .2);
  L limiter;
  limiter.init(false);
  auto cmd = limiter.limit(2., -9.);
  near(cmd[0], .4); near(cmd[1], -3.);
  cmd = limiter.limit(-2., 9.);
  near(cmd[0], -.4); near(cmd[1], 3.);
  limiter.init(true, .3, 3.5, .7, 5.2, .4, 3.);
  assert(limiter.isEnabled());
  std::cout << "Passed: speed caps, signed acceleration/braking, reversal through zero, nonpositive dt\n";
}
