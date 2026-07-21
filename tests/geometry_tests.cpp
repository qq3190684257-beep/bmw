#include <cassert>
#include <cmath>
#include <iostream>
#include <vector>

#include "../src/Geometry.hpp"
#include "../src/PostCollisionPhysics.hpp"

namespace {

bool near(float actual, float expected, float tolerance = 1.0e-4f) {
    return std::fabs(actual - expected) <= tolerance;
}

float degrees(float radians) {
    constexpr float kPi = 3.14159265358979323846f;
    return radians * 180.0f / kPi;
}

poollab::Vec2 incomingAtTopRail(float incidenceDegrees) {
    constexpr float kPi = 3.14159265358979323846f;
    const float radians = incidenceDegrees * kPi / 180.0f;
    return {std::sin(radians), std::cos(radians)};
}

float outgoingAngleAtTopRail(poollab::Vec2 direction) {
    direction = poollab::normalized(direction);
    const poollab::Vec2 inwardNormal{0.0f, -1.0f};
    return degrees(std::atan2(std::fabs(direction.x),
                              std::max(0.0f, poollab::dot(direction,
                                                         inwardNormal))));
}

}  // namespace

int main() {
    using namespace poollab;

    const Bounds2 outer{{-1.3335f, -0.7963796f},
                        {1.3335f, 0.5371203f}};
    const float radius = 0.04123377f;
    const Vec2 cue{-0.66675f, -0.1296296f};
    const Vec2 aim{0.3595651f, 0.93312f};
    const std::vector<Ball2> balls{{0, cue, true}};

    const Prediction centerBoundary = predict(
        balls, 0, aim, radius, outer, false, 0.0f, 1, 0.0f, true, 1.0f);
    assert(centerBoundary.cueApproachRoute.count == 2);
    const Segment2 firstCenter = centerBoundary.cueApproachRoute.segments[0];
    assert(firstCenter.valid);
    assert(near(firstCenter.b.y, outer.max.y - radius));
    assert(near(firstCenter.b.x, -0.42572f, 2.0e-4f));

    const Prediction outerBoundary = predict(
        balls, 0, aim, radius, outer, false, 0.0f, 1, 0.0f, true, 0.0f);
    assert(outerBoundary.cueApproachRoute.count == 2);
    const Segment2 firstOuter = outerBoundary.cueApproachRoute.segments[0];
    assert(firstOuter.valid);
    assert(near(firstOuter.b.y, outer.max.y));
    assert(near(firstOuter.b.x, -0.40983f, 2.0e-4f));

    // The supplied device trace hit y=0.496899 at the first cushion.  The
    // center boundary is within roughly 0.001 world units; the outer boundary
    // is one complete ball radius too far away.
    const float observedY = 0.49689934f;
    assert(std::fabs(firstCenter.b.y - observedY) < 0.002f);
    assert(std::fabs(firstOuter.b.y - observedY) > 0.03f);

    // Full-force device traces show that cushion friction makes the outgoing
    // direction closer to the normal by a force-dependent ratio, not by one
    // fixed angular subtraction.  One restitution/friction pair must reproduce
    // both observed shots.
    const Bounds2 unitBounds{{-1.0f, -1.0f}, {1.0f, 1.0f}};
    const Vec2 topRailPoint{0.0f, 1.0f};
    const Vec2 shallowReflection = reflectedAtRail(
        incomingAtTopRail(21.07f), topRailPoint, unitBounds);
    const Vec2 steepReflection = reflectedAtRail(
        incomingAtTopRail(36.14f), topRailPoint, unitBounds);
    assert(near(outgoingAngleAtTopRail(shallowReflection), 15.58f, 0.10f));
    assert(near(outgoingAngleAtTopRail(steepReflection), 33.32f, 0.10f));

    // Manual angle correction is now an extra trim on top of the physical
    // reflection rather than the primary cushion model.
    const Vec2 trimmedReflection = reflectedAtRail(
        incomingAtTopRail(36.14f), topRailPoint, unitBounds, 1.25f);
    assert(near(outgoingAngleAtTopRail(trimmedReflection),
                outgoingAngleAtTopRail(steepReflection) + 1.25f, 0.01f));
    const Vec2 fineTrimmedReflection = reflectedAtRail(
        incomingAtTopRail(36.14f), topRailPoint, unitBounds, 0.09f);
    assert(near(outgoingAngleAtTopRail(fineTrimmedReflection),
                outgoingAngleAtTopRail(steepReflection) + 0.09f, 0.01f));

    const Vec2 normalReflection = reflectedAtRail(
        {0.0f, 1.0f}, topRailPoint, unitBounds);
    assert(near(normalReflection.x, 0.0f));
    assert(near(normalReflection.y, -1.0f));

    // 2026-07-21 zero-spin shot: native EX ends at the cue/7 contact.
    // The measured incoming velocity then transfers through 7 -> 3. The cue
    // endpoint and primary object endpoint must reproduce the recorded stops.
    std::vector<Ball2> collisionBalls(16);
    for (int i = 0; i < 16; ++i)
        collisionBalls[static_cast<std::size_t>(i)] = {i, {}, true};
    collisionBalls[0].center = {0.5076714f, -0.0955051f};
    collisionBalls[3].center = {0.003139236f, 0.02761425f};
    collisionBalls[7].center = {0.07829323f, -0.2923235f};
    // Keep unrelated balls away from the calibrated collision chain.
    for (int i = 1; i < 16; ++i) {
        if (i != 3 && i != 7)
            collisionBalls[static_cast<std::size_t>(i)].center =
                {0.75f + 0.02f * i, 0.25f};
    }
    const Vec2 measuredImpact{0.095828607f, -0.37684895f};
    const Vec2 measuredIncoming = normalized(
        Vec2{0.095828607f - 0.1454207f,
             -0.37684895f - (-0.4757639f)}) * 1.12352f;
    const PostCollisionPrediction post = predictPostCollision(
        collisionBalls, 0, 7, measuredImpact, measuredIncoming,
        radius, outer);
    assert(post.valid);
    assert(post.firstTargetIndex == 7);
    assert(post.objectCollisionCount == 1);
    assert(post.objectRailCount == 1);
    assert(post.cueRoute.count >= 3);
    assert(post.objectRoute.count >= 5);
    const Vec2 cueStop = post.cueRoute.points[
        static_cast<std::size_t>(post.cueRoute.count - 1)];
    const Vec2 objectStop = post.objectRoute.points[
        static_cast<std::size_t>(post.objectRoute.count - 1)];
    assert(length(cueStop - Vec2{-0.2193161f, -0.1519356f}) < 0.015f);
    assert(length(objectStop - Vec2{-0.1815301f, 0.4298801f}) < 0.030f);

    // Native after-contact vectors must control the two post-collision
    // directions when available; they are not reused as incoming speed.
    const Vec2 nativeCueAfter{0.12f, -0.31f};
    const Vec2 nativeObjectAfter{-0.44f, 0.18f};
    const PostCollisionPrediction nativePost = predictPostCollision(
        collisionBalls, 0, 7, measuredImpact, measuredIncoming,
        radius, outer, nativeCueAfter, nativeObjectAfter);
    assert(nativePost.valid);
    assert(nativePost.nativeCueAfterVelocityUsed);
    assert(nativePost.nativeObjectAfterVelocityUsed);
    assert(near(nativePost.incomingSpeed, length(measuredIncoming), 1.0e-6f));

    const float force = 2.523514f;
    const float launchSpeed = launchSpeedFromForce(force);
    // 2.523514 lies just above the 2.519000 calibration point, so this
    // assertion checks the piecewise interpolation rather than the old
    // single-multiplier value.
    assert(near(launchSpeed, 2.8573f, 0.002f));
    assert(near(launchSpeedFromForce(0.977261f), 1.080019f, 0.001f));
    assert(launchSpeedFromForce(0.977261f) < force * 1.1218f);
    assert(stationaryBallStopDistance(launchSpeed) > 10.0f);

    std::cout << "geometry tests passed\n";
    return 0;
}
