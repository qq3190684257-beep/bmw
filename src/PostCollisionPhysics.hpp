#pragma once

#include "Geometry.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

namespace poollab {

// Calibrated from probe-20260721-085251 (game 3.61.0, zero user spin).
// The trace exposes the two friction phases used by the native simulation:
//   sliding: about 3.672 world units/s^2
//   rolling: about 0.1163 world units/s^2
// A stationary object ball settles at 5/7 of its impact speed, matching the
// solid-sphere sliding-to-rolling transition and the measured 7 -> 3 chain.
constexpr float kSlidingDeceleration = 3.672f;
constexpr float kRollingDeceleration = 0.1163f;
constexpr float kBallCollisionRestitution = 0.734f;
constexpr float kBallNormalTransfer =
    (1.0f + kBallCollisionRestitution) * 0.5f;
constexpr float kRollingRailSpeedRetention = 0.576f;

// fForce is not a single linear world-speed scale. These zero-spin straight
// shots calibrate the low-force and high-force portions of the curve.
struct ForceCalibrationPoint {
    float force;
    float speed;
};

constexpr std::array<ForceCalibrationPoint, 6> kForceCalibration{{
    {0.000000f, 0.000000f},
    {0.977261f, 1.080019f},
    {1.254219f, 1.385812f},
    {2.519000f, 2.852000f},
    {3.775706f, 4.317741f},
    {5.050000f, 5.726350f},
}};

inline float launchSpeedFromForce(float force) {
    if (!std::isfinite(force) || force <= 0.0f) return 0.0f;
    for (std::size_t i = 1; i < kForceCalibration.size(); ++i) {
        const ForceCalibrationPoint& a = kForceCalibration[i - 1];
        const ForceCalibrationPoint& b = kForceCalibration[i];
        if (force > b.force) continue;
        const float span = b.force - a.force;
        if (span <= kEpsilon) return b.speed;
        const float t = (force - a.force) / span;
        return a.speed + (b.speed - a.speed) * t;
    }
    const ForceCalibrationPoint& a = kForceCalibration[kForceCalibration.size() - 2];
    const ForceCalibrationPoint& b = kForceCalibration[kForceCalibration.size() - 1];
    const float span = b.force - a.force;
    return b.speed + (force - b.force) * (b.speed - a.speed) / span;
}

// Natural-roll cue-ball coefficients after the first object collision.  They
// retain the measured tangent component and the forward-roll component instead
// of forcing the second route to be a straight ideal-collision ray.
constexpr float kCueTangentRetention = 0.660f;
constexpr float kCueNormalFollowRetention = 0.197f;
constexpr float kCueCurveDistancePerIncomingSpeed = 0.0267f;

constexpr std::size_t kPostCollisionPointCapacity = 128;

struct WorldPolyline {
    std::array<Vec2, kPostCollisionPointCapacity> points{};
    int count = 0;
};

struct PostCollisionPrediction {
    bool valid = false;
    bool incomingSpeedFromNative = false;
    bool nativeCueAfterVelocityUsed = false;
    bool nativeObjectAfterVelocityUsed = false;
    int firstTargetIndex = -1;
    Vec2 impactPoint;
    Vec2 incomingVelocity;
    float incomingSpeed = 0.0f;
    float fallbackIncomingSpeed = 0.0f;
    float probedIncomingSpeed = 0.0f;
    WorldPolyline cueRoute;
    WorldPolyline objectRoute;
    int objectCollisionCount = 0;
    int objectRailCount = 0;
};

inline bool appendPoint(WorldPolyline& route, Vec2 point,
                        float minimumSeparation = 1.0e-5f) {
    if (!finite(point) || route.count >=
        static_cast<int>(kPostCollisionPointCapacity)) return false;
    if (route.count > 0 && lengthSquared(
            point - route.points[static_cast<std::size_t>(route.count - 1)]) <=
            minimumSeparation * minimumSeparation) return true;
    route.points[static_cast<std::size_t>(route.count++)] = point;
    return true;
}

inline float worldPolylineLength(const WorldPolyline& route) {
    const int count = std::max(
        0, std::min(static_cast<int>(kPostCollisionPointCapacity), route.count));
    float total = 0.0f;
    for (int i = 1; i < count; ++i) {
        total += length(route.points[static_cast<std::size_t>(i)] -
                        route.points[static_cast<std::size_t>(i - 1)]);
    }
    return total;
}

inline float slidingTransitionTime(float initialSpeed) {
    if (!std::isfinite(initialSpeed) || initialSpeed <= kEpsilon) return 0.0f;
    return 2.0f * initialSpeed / (7.0f * kSlidingDeceleration);
}

inline float slidingTransitionDistance(float initialSpeed) {
    const float time = slidingTransitionTime(initialSpeed);
    return initialSpeed * time -
           0.5f * kSlidingDeceleration * time * time;
}

inline float rollingSpeedAfterSliding(float initialSpeed) {
    return std::max(0.0f, initialSpeed * (5.0f / 7.0f));
}

inline float rollingStopDistance(float speed) {
    if (!std::isfinite(speed) || speed <= kEpsilon) return 0.0f;
    return speed * speed / (2.0f * kRollingDeceleration);
}

inline float stationaryBallStopDistance(float initialSpeed) {
    return slidingTransitionDistance(initialSpeed) +
           rollingStopDistance(rollingSpeedAfterSliding(initialSpeed));
}

inline float stationaryBallSpeedAfterTravel(float initialSpeed,
                                             float travel) {
    if (!std::isfinite(initialSpeed) || initialSpeed <= kEpsilon ||
        !std::isfinite(travel) || travel < 0.0f) return 0.0f;
    const float slideDistance = slidingTransitionDistance(initialSpeed);
    if (travel <= slideDistance + kEpsilon) {
        const float discriminant = std::max(
            0.0f, initialSpeed * initialSpeed -
                      2.0f * kSlidingDeceleration * travel);
        return std::sqrt(discriminant);
    }
    const float rollingSpeed = rollingSpeedAfterSliding(initialSpeed);
    const float rollingTravel = travel - slideDistance;
    return std::sqrt(std::max(
        0.0f, rollingSpeed * rollingSpeed -
                  2.0f * kRollingDeceleration * rollingTravel));
}

inline Vec2 mirrorAtRail(Vec2 direction, Vec2 railPoint,
                         const Bounds2& innerBounds) {
    direction = normalized(direction);
    const float tolerance = 1.0e-3f * std::max(
        innerBounds.max.x - innerBounds.min.x,
        innerBounds.max.y - innerBounds.min.y);
    if (std::fabs(railPoint.x - innerBounds.min.x) <= tolerance ||
        std::fabs(railPoint.x - innerBounds.max.x) <= tolerance)
        direction.x = -direction.x;
    if (std::fabs(railPoint.y - innerBounds.min.y) <= tolerance ||
        std::fabs(railPoint.y - innerBounds.max.y) <= tolerance)
        direction.y = -direction.y;
    return normalized(direction);
}

inline int firstBallOnLeg(const std::vector<Ball2>& balls,
                          int movingBallIndex,
                          Vec2 origin,
                          Vec2 direction,
                          float radius,
                          float maximumTravel,
                          const std::array<bool, 32>& alreadyTransferred,
                          float& travel) {
    int selected = -1;
    travel = std::numeric_limits<float>::infinity();
    direction = normalized(direction);
    const float collisionRadius = 2.0f * radius;
    const float collisionRadiusSquared = collisionRadius * collisionRadius;
    for (std::size_t i = 0; i < balls.size(); ++i) {
        if (static_cast<int>(i) == movingBallIndex || !balls[i].active ||
            (i < alreadyTransferred.size() && alreadyTransferred[i])) continue;
        const Vec2 relative = balls[i].center - origin;
        const float projection = dot(relative, direction);
        if (projection <= kEpsilon) continue;
        const float perpendicularSquared = std::max(
            0.0f, lengthSquared(relative) - projection * projection);
        if (perpendicularSquared > collisionRadiusSquared) continue;
        const float candidate = projection - std::sqrt(std::max(
            0.0f, collisionRadiusSquared - perpendicularSquared));
        if (candidate > kEpsilon && candidate <= maximumTravel + kEpsilon &&
            candidate < travel) {
            travel = candidate;
            selected = static_cast<int>(i);
        }
    }
    return selected;
}

inline void appendCueCurve(WorldPolyline& route,
                           Vec2 impactPoint,
                           Vec2 immediateDirection,
                           Vec2 rollingDirection,
                           float curveDistance) {
    appendPoint(route, impactPoint);
    immediateDirection = normalized(immediateDirection);
    rollingDirection = normalized(rollingDirection);
    if (curveDistance <= kEpsilon ||
        lengthSquared(rollingDirection) <= kEpsilon) return;
    if (lengthSquared(immediateDirection) <= kEpsilon)
        immediateDirection = rollingDirection;
    const Vec2 control = impactPoint + immediateDirection *
        (curveDistance * 0.62f);
    const Vec2 end = impactPoint + rollingDirection * curveDistance;
    for (int sample = 1; sample <= 5; ++sample) {
        const float t = static_cast<float>(sample) / 5.0f;
        const float oneMinusT = 1.0f - t;
        appendPoint(route,
                    impactPoint * (oneMinusT * oneMinusT) +
                    control * (2.0f * oneMinusT * t) + end * (t * t));
    }
}

inline void appendRollingRoute(WorldPolyline& route,
                               Vec2 start,
                               Vec2 direction,
                               float speed,
                               const Bounds2& innerBounds,
                               int maximumRails = 4) {
    direction = normalized(direction);
    if (!innerBounds.valid() || lengthSquared(direction) < kEpsilon ||
        speed <= kEpsilon) return;
    appendPoint(route, start);
    for (int rail = 0; rail <= maximumRails && speed > kEpsilon; ++rail) {
        const float stopDistance = rollingStopDistance(speed);
        const float railDistance = rayToInnerBounds(start, direction, innerBounds);
        if (railDistance <= kEpsilon || stopDistance <= railDistance + kEpsilon) {
            appendPoint(route, start + direction * stopDistance);
            return;
        }
        const Vec2 railPoint = start + direction * railDistance;
        appendPoint(route, railPoint);
        speed = std::sqrt(std::max(
            0.0f, speed * speed -
                      2.0f * kRollingDeceleration * railDistance));
        speed *= kRollingRailSpeedRetention;
        direction = mirrorAtRail(direction, railPoint, innerBounds);
        start = railPoint;
    }
}

inline void appendPrimaryObjectChain(WorldPolyline& route,
                                     const std::vector<Ball2>& balls,
                                     int firstTargetIndex,
                                     Vec2 initialDirection,
                                     float initialSpeed,
                                     float radius,
                                     const Bounds2& innerBounds,
                                     int& collisionCount,
                                     int& railCount) {
    collisionCount = 0;
    railCount = 0;
    if (firstTargetIndex < 0 ||
        static_cast<std::size_t>(firstTargetIndex) >= balls.size() ||
        !balls[static_cast<std::size_t>(firstTargetIndex)].active ||
        initialSpeed <= kEpsilon || radius <= kEpsilon ||
        !innerBounds.valid()) return;

    int movingBall = firstTargetIndex;
    Vec2 start = balls[static_cast<std::size_t>(movingBall)].center;
    Vec2 direction = normalized(initialDirection);
    float speed = initialSpeed;
    bool startsSliding = true;
    std::array<bool, 32> transferred{};
    if (static_cast<std::size_t>(movingBall) < transferred.size())
        transferred[static_cast<std::size_t>(movingBall)] = true;
    appendPoint(route, start);

    for (int event = 0; event < 12 && speed > kEpsilon; ++event) {
        const float stopDistance = startsSliding
            ? stationaryBallStopDistance(speed) : rollingStopDistance(speed);
        const float railDistance = rayToInnerBounds(start, direction, innerBounds);
        const float maximumStraightTravel = std::min(stopDistance, railDistance);
        float ballTravel = std::numeric_limits<float>::infinity();
        const int nextBall = firstBallOnLeg(
            balls, movingBall, start, direction, radius,
            maximumStraightTravel, transferred, ballTravel);

        if (nextBall >= 0) {
            const Vec2 movingCenter = start + direction * ballTravel;
            appendPoint(route, movingCenter);
            const float impactSpeed = startsSliding
                ? stationaryBallSpeedAfterTravel(speed, ballTravel)
                : std::sqrt(std::max(
                    0.0f, speed * speed -
                              2.0f * kRollingDeceleration * ballTravel));
            const Vec2 normal = normalized(
                balls[static_cast<std::size_t>(nextBall)].center - movingCenter);
            const float normalSpeed = std::max(0.0f, dot(direction, normal)) *
                                      impactSpeed;
            speed = kBallNormalTransfer * normalSpeed;
            movingBall = nextBall;
            start = balls[static_cast<std::size_t>(movingBall)].center;
            direction = normal;
            startsSliding = true;
            if (static_cast<std::size_t>(movingBall) < transferred.size())
                transferred[static_cast<std::size_t>(movingBall)] = true;
            appendPoint(route, start);
            ++collisionCount;
            continue;
        }

        if (stopDistance <= railDistance + kEpsilon) {
            appendPoint(route, start + direction * stopDistance);
            return;
        }

        const Vec2 railPoint = start + direction * railDistance;
        appendPoint(route, railPoint);
        const float speedAtRail = startsSliding
            ? stationaryBallSpeedAfterTravel(speed, railDistance)
            : std::sqrt(std::max(
                0.0f, speed * speed -
                          2.0f * kRollingDeceleration * railDistance));
        speed = speedAtRail * kRollingRailSpeedRetention;
        direction = mirrorAtRail(direction, railPoint, innerBounds);
        start = railPoint;
        startsSliding = false;
        ++railCount;
    }
}

inline PostCollisionPrediction predictPostCollision(
        const std::vector<Ball2>& balls,
        int cueIndex,
        int targetIndex,
        Vec2 cueImpact,
        Vec2 incomingVelocity,
        float radius,
        Bounds2 tableBounds,
        Vec2 nativeCueAfterVelocity = {},
        Vec2 nativeObjectAfterVelocity = {}) {
    PostCollisionPrediction result;
    if (cueIndex < 0 || targetIndex < 0 || cueIndex == targetIndex ||
        static_cast<std::size_t>(cueIndex) >= balls.size() ||
        static_cast<std::size_t>(targetIndex) >= balls.size() ||
        !balls[static_cast<std::size_t>(cueIndex)].active ||
        !balls[static_cast<std::size_t>(targetIndex)].active ||
        radius <= kEpsilon || !finite(cueImpact) ||
        !finite(incomingVelocity) || !tableBounds.valid()) return result;

    const float incomingSpeed = length(incomingVelocity);
    if (incomingSpeed <= kEpsilon) return result;
    const Vec2 collisionNormal = normalized(
        balls[static_cast<std::size_t>(targetIndex)].center - cueImpact);
    if (lengthSquared(collisionNormal) <= kEpsilon) return result;
    const float normalSpeed = std::max(0.0f,
        dot(incomingVelocity, collisionNormal));
    if (normalSpeed <= kEpsilon) return result;

    Bounds2 inner = tableBounds;
    inner.min = inner.min + Vec2{radius, radius};
    inner.max = inner.max - Vec2{radius, radius};
    if (!inner.valid()) return result;

    result.valid = true;
    result.firstTargetIndex = targetIndex;
    result.impactPoint = cueImpact;
    result.incomingVelocity = incomingVelocity;
    result.incomingSpeed = incomingSpeed;

    const Vec2 normalVelocity = collisionNormal * normalSpeed;
    const Vec2 tangentVelocity = incomingVelocity - normalVelocity;
    Vec2 immediateCueVelocity = incomingVelocity -
        normalVelocity * kBallNormalTransfer;
    Vec2 rollingCueVelocity =
        tangentVelocity * kCueTangentRetention +
        normalVelocity * kCueNormalFollowRetention;
    if (finite(nativeCueAfterVelocity) &&
        lengthSquared(nativeCueAfterVelocity) > kEpsilon) {
        immediateCueVelocity = nativeCueAfterVelocity;
        rollingCueVelocity = nativeCueAfterVelocity;
        result.nativeCueAfterVelocityUsed = true;
    }
    const float cueRollingSpeed = length(rollingCueVelocity);
    // A one-point route is a valid landing prediction: on a stop-like hit the
    // cue ball can finish at the impact point without producing a second leg.
    appendPoint(result.cueRoute, cueImpact);
    if (cueRollingSpeed > kEpsilon) {
        const float curveDistance = std::min(
            rollingStopDistance(cueRollingSpeed) * 0.20f,
            incomingSpeed * kCueCurveDistancePerIncomingSpeed);
        appendCueCurve(result.cueRoute, cueImpact, immediateCueVelocity,
                       rollingCueVelocity, curveDistance);
        const Vec2 curveEnd = result.cueRoute.count > 0
            ? result.cueRoute.points[static_cast<std::size_t>(
                result.cueRoute.count - 1)] : cueImpact;
        appendRollingRoute(result.cueRoute, curveEnd, rollingCueVelocity,
                           cueRollingSpeed, inner);
    }

    Vec2 objectDirection = collisionNormal;
    float objectSpeed = kBallNormalTransfer * normalSpeed;
    if (finite(nativeObjectAfterVelocity) &&
        lengthSquared(nativeObjectAfterVelocity) > kEpsilon) {
        objectDirection = normalized(nativeObjectAfterVelocity);
        objectSpeed = length(nativeObjectAfterVelocity);
        result.nativeObjectAfterVelocityUsed = true;
    }
    appendPrimaryObjectChain(
        result.objectRoute, balls, targetIndex, objectDirection,
        objectSpeed, radius, inner,
        result.objectCollisionCount, result.objectRailCount);
    return result;
}

}  // namespace poollab
