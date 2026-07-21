#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>

#include "../src/IL2CPPBridge.hpp"

namespace {

bool near(float actual, float expected, float tolerance = 1.0e-5f) {
    return std::fabs(actual - expected) <= tolerance;
}

}  // namespace

int main() {
    using namespace poollab;

    static_assert(kLegacyNativeTrajectoryCapacity == 90,
                  "legacy preview ABI requires exactly 90 samples");
    static_assert(kBallCapacity == 32,
                  "native PhysicsCoordinate supports up to 32 balls");
    static_assert(isSnookerBallType(kBallTypeShortSnooker));
    static_assert(isSnookerBallType(kBallTypeSnooker));
    static_assert(!isSnookerBallType(kBallTypeEightBall));

    std::array<Vec2, kLegacyNativeTrajectoryCapacity> zeroTerminated{};
    zeroTerminated[0] = {-1.0f, 0.0f};
    zeroTerminated[1] = {0.0f, 1.0f};
    zeroTerminated[2] = {1.0f, 0.0f};
    assert(legacyScannedPointCount(zeroTerminated) == 3);
    assert(near(legacyMaximumChordResidual(zeroTerminated, 3), 1.0f));

    std::array<Vec2, kLegacyNativeTrajectoryCapacity> straight{};
    const float nan = std::numeric_limits<float>::quiet_NaN();
    for (Vec2& point : straight) point = {nan, nan};
    straight[0] = {-1.0f, -1.0f};
    straight[1] = {0.0f, 0.0f};
    straight[2] = {1.0f, 1.0f};
    assert(legacyScannedPointCount(straight) == 3);
    assert(near(legacyMaximumChordResidual(straight, 3), 0.0f));

    std::array<Vec2, kLegacyNativeTrajectoryCapacity> repeated{};
    for (Vec2& point : repeated) point = {nan, nan};
    repeated[0] = {-1.0f, 0.0f};
    repeated[1] = {0.0f, 0.5f};
    for (int i = 2; i < 8; ++i) repeated[static_cast<std::size_t>(i)] = {1.0f, 1.0f};
    assert(legacyScannedPointCount(repeated) == 3);

    std::cout << "native probe tests passed\n";
    return 0;
}
