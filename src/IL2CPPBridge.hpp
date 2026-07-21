#pragma once

#include "Geometry.hpp"
#include "PostCollisionPhysics.hpp"

#include <array>
#include <cstddef>
#include <string>

namespace poollab {

// PhysicsCoordinate.BALL_NUM_MAX is 32. Eight-ball uses indices 0..15 while
// short/full Snooker can expose additional balls through the same transform
// array and native PhysicsCall API.
constexpr std::size_t kBallCapacity = 32;
constexpr int kBallTypeEightBall = 5;
constexpr int kBallTypeShortSnooker = 15;
constexpr int kBallTypeSnooker = 16;

constexpr bool isSnookerBallType(int ballType) {
    return ballType == kBallTypeShortSnooker ||
           ballType == kBallTypeSnooker;
}

struct RuntimeBall {
    int index = -1;
    std::string name;
    std::string typeName;
    Vec3 world;
    Vec3 transformWorld;
    Vec3 screenPixels;
    bool visible = false;
    bool transformVisible = false;
};

struct RuntimeScreenSegment {
    Vec3 a;
    Vec3 b;
    bool visible = false;
};

struct RuntimeScreenRoute {
    std::array<RuntimeScreenSegment, kTrajectoryRouteCapacity> segments{};
    int count = 0;
};

constexpr std::size_t kLegacyNativeTrajectoryCapacity = 90;

struct RuntimeScreenPolyline {
    std::array<Vec3, kLegacyNativeTrajectoryCapacity> points{};
    int count = 0;
    bool visible = false;
};

struct RuntimePocket {
    Vec3 world;
    Vec3 screenPixels;
    bool visible = false;
};

constexpr std::size_t kPhysicsEdgeCapacity = 32;
constexpr std::size_t kPhysicsHoleCapacity = 8;

struct RuntimePhysicsEdge {
    Vec2 start;
    Vec2 end;
    Vec3 startScreen;
    Vec3 endScreen;
    bool visible = false;
};

struct RuntimePhysicsHole {
    int index = -1;
    Vec2 center;
    Vec2 leftOffset;
    Vec2 rightOffset;
    Vec2 leftEdge;
    Vec2 rightEdge;
    Vec2 leftDirection;
    Vec2 rightDirection;
    Vec3 centerScreen;
    Vec3 leftEdgeScreen;
    Vec3 rightEdgeScreen;
    bool visible = false;
};

struct RuntimePhysicsConfig {
    bool modelFound = false;
    bool available = false;
    int edgeCount = 0;
    int edgeCaptured = 0;
    int holeCount = 0;
    int holeCaptured = 0;
    bool nativeLineMethodsFound = false;
    bool nativeLinesAvailable = false;
    int nativeEdgeLineCount = 0;
    int nativeEdgeLineCaptured = 0;
    int nativeHoleLineCount = 0;
    int nativeHoleLineCaptured = 0;
    float coordinateWidth = 0.0f;
    float coordinateHeight = 0.0f;
    float coordinateScale = 1.0f;
    Vec2 coordinateOffset;
    float ballScreenRadius = 0.0f;
    bool coordinateBoundsReady = false;
    bool usedPocketFallback = false;
    bool boundsProjected = false;
    std::array<Vec3, 4> outerBoundsScreen{};
    std::array<Vec3, 4> railBoundsScreen{};
    std::array<RuntimePhysicsEdge, kPhysicsEdgeCapacity> edges{};
    std::array<RuntimePhysicsHole, kPhysicsHoleCapacity> holes{};
    std::array<RuntimePhysicsEdge, kPhysicsEdgeCapacity> nativeEdgeLines{};
    std::array<RuntimePhysicsEdge, kPhysicsEdgeCapacity> nativeHoleLines{};
};

constexpr std::size_t kProbeArrayCapacity = 32;

struct RuntimeProbe {
    bool available = false;
    bool shootInfoAvailable = false;
    bool isShowLine = false;
    float forceValue = 0.0f;
    float forceDuration = 0.0f;
    float shotForce = 0.0f;
    float addedForce = 0.0f;
    float shootValue = 0.0f;
    float shootForce = 0.0f;
    float xSpin = 0.0f;
    float ySpin = 0.0f;
    float xMouse = 0.0f;
    float yMouse = 0.0f;
    int degree = 0;
    int randIc = 0;
    float trace = 0.0f;
    int lineDataCount = 0;
    int lineDataCaptured = 0;
    std::array<Vec2, kProbeArrayCapacity> lineData{};
    int xPosCount = 0;
    int xPosCaptured = 0;
    std::array<float, kProbeArrayCapacity> xPos{};
    int yPosCount = 0;
    int yPosCaptured = 0;
    std::array<float, kProbeArrayCapacity> yPos{};
};

// PhysicsEx.PhysicsCall::getAllCollisionDataFinalSimple returns at most four
// affected-ball routes. Keep a bounded copy in native memory so logging never
// retains pointers owned by the game's physics engine.
constexpr std::size_t kNativePhysicsRouteCapacity = 4;
constexpr std::size_t kNativePhysicsTrajectoryCapacity = 256;
constexpr std::size_t kNativePhysicsCollisionCapacity = 64;

struct RuntimeNativePhysicsRoute {
    bool valid = false;
    int ballIndex = -1;
    int trajectoryPointCount = 0;
    int trajectoryPointCaptured = 0;
    std::array<Vec2, kNativePhysicsTrajectoryCapacity> trajectory{};
    int collisionPointCount = 0;
    int collisionPointCaptured = 0;
    std::array<Vec2, kNativePhysicsCollisionCapacity> collisionPoints{};
    int collisionBallCount = 0;
    int collisionBallCaptured = 0;
    std::array<int, kNativePhysicsCollisionCapacity> collisionBalls{};
    int rawXTrajectorySize = 0;
    int rawYTrajectorySize = 0;
    int rawXCollisionSize = 0;
    int rawYCollisionSize = 0;
    int rawCollisionBallSize = 0;
};

// Small, pointer-free snapshot used to compare the possible IL2CPP value-type
// return layouts without retaining or dereferencing engine-owned buffers.
struct RuntimeNativeRouteCandidate {
    bool captured = false;
    int rawBallIndex = 0;
    bool selfValid = false;
    int ballIndex = -1;
    int xTrajectorySize = 0;
    int yTrajectorySize = 0;
    int xCollisionSize = 0;
    int yCollisionSize = 0;
    int collisionBallSize = 0;
    bool getterAttempted = false;
    bool getterAvailable = false;
    bool getterValid = false;
    int getterTrajectoryCount = -1;
    int getterCollisionCount = -1;
};

struct RuntimeNativeEngineBall {
    bool positionValid = false;
    bool speedValid = false;
    Vec2 position;
    Vec2 speed;
};

// PhysicsEx.PhysicsCall::getAllCollisionData/GetAllCollisionDataEx expose the
// legacy fixed-size native preview.  Keep the original 90 samples intact so a
// curved post-cushion path is never collapsed to one endpoint-to-endpoint ray.
struct RuntimeLegacyCollisionProbe {
    bool methodFound = false;
    bool callAttempted = false;
    bool directSretUsed = false;
    int directSretChangedBytes = -1;
    int requestedCollisionCount = 0;
    int reportedPointCount = -1;
    int scannedPointCount = 0;
    int trajectoryPointCount = 0;
    int trajectoryPointCaptured = 0;
    int collisionBall = -1;
    float maximumChordResidual = 0.0f;
    bool valid = false;
    std::string status = "not_bound";
    std::array<Vec2, kLegacyNativeTrajectoryCapacity> trajectory{};
};

inline int legacyScannedPointCount(
        const std::array<Vec2, kLegacyNativeTrajectoryCapacity>& points) {
    int count = 0;
    for (const Vec2 point : points) {
        if (!finite(point) || std::fabs(point.x) > 10000.0f ||
            std::fabs(point.y) > 10000.0f) break;
        ++count;
    }
    int zeroSuffix = count;
    while (zeroSuffix > 0) {
        const Vec2 point = points[static_cast<std::size_t>(zeroSuffix - 1)];
        if (std::fabs(point.x) > 1.0e-7f || std::fabs(point.y) > 1.0e-7f)
            break;
        --zeroSuffix;
    }
    if (count - zeroSuffix >= 4) count = zeroSuffix;
    if (count >= 4) {
        const Vec2 last = points[static_cast<std::size_t>(count - 1)];
        int repeatStart = count - 1;
        while (repeatStart > 0) {
            const Vec2 previous = points[static_cast<std::size_t>(repeatStart - 1)];
            if (std::fabs(previous.x - last.x) > 1.0e-7f ||
                std::fabs(previous.y - last.y) > 1.0e-7f) break;
            --repeatStart;
        }
        if (count - repeatStart >= 4) count = repeatStart + 1;
    }
    return count;
}

inline float legacyMaximumChordResidual(
        const std::array<Vec2, kLegacyNativeTrajectoryCapacity>& points,
        int count) {
    count = std::max(0, std::min(
        static_cast<int>(kLegacyNativeTrajectoryCapacity), count));
    if (count < 3) return 0.0f;
    const Vec2 a = points[0];
    const Vec2 chord = points[static_cast<std::size_t>(count - 1)] - a;
    const float chordLength = length(chord);
    if (chordLength <= kEpsilon) return 0.0f;
    float maximum = 0.0f;
    for (int i = 1; i + 1 < count; ++i) {
        const Vec2 relative = points[static_cast<std::size_t>(i)] - a;
        const float areaTwice = std::fabs(chord.x * relative.y -
                                          chord.y * relative.x);
        maximum = std::max(maximum, areaTwice / chordLength);
    }
    return maximum;
}

struct RuntimeNativePhysicsProbe {
    bool classFound = false;
    bool methodFound = false;
    bool stateMethodsFound = false;
    bool stateAttempted = false;
    bool stateAvailable = false;
    int engineBallCount = -1;
    int engineBallType = -1;
    int enginePositionCount = 0;
    int transformComparisonCount = 0;
    float transformMaximumDelta = 0.0f;
    std::array<RuntimeNativeEngineBall, kBallCapacity> engineBalls{};
    bool modeFieldsFound = false;
    bool modeValuesAvailable = false;
    bool useEx = false;
    bool useExV0 = false;
    bool useExV1 = false;
    bool bothMode = false;
    bool legacyMethodFound = false;
    bool legacyExMethodFound = false;
    bool firstCollisionSpeedMethodFound = false;
    bool firstCollisionSpeedAttempted = false;
    bool firstCollisionSpeedAvailable = false;
    int firstCollisionSpeedBallIndex = 0;
    Vec2 firstCollisionSpeedRaw;
    Vec2 firstCollisionSpeedWorld;
    bool targetCollisionSpeedAttempted = false;
    bool targetCollisionSpeedAvailable = false;
    int targetCollisionSpeedBallIndex = -1;
    Vec2 targetCollisionSpeedRaw;
    Vec2 targetCollisionSpeedWorld;
    RuntimeLegacyCollisionProbe legacy;
    RuntimeLegacyCollisionProbe legacyEx;
    bool managedParserFound = false;
    bool managedValidCountAvailable = false;
    int managedValidCount = -1;
    bool valueBoxAvailable = false;
    bool managedArrayMethodFound = false;
    bool managedArrayAvailable = false;
    int managedArrayLength = -1;
    bool directSretUsed = false;
    int directSretChangedBytes = -1;
    int directCollisionValidCount = -1;
    int directValidCount = 0;
    std::array<RuntimeNativeRouteCandidate, kNativePhysicsRouteCapacity>
        resultCandidates{};
    std::array<RuntimeNativeRouteCandidate, kNativePhysicsRouteCapacity>
        arrayCandidates{};
    std::array<RuntimeNativeRouteCandidate, kNativePhysicsRouteCapacity>
        getDataBaseCandidates{};
    std::array<RuntimeNativeRouteCandidate, kNativePhysicsRouteCapacity>
        getDataPayloadCandidates{};
    std::array<bool, kNativePhysicsRouteCapacity> getDataObjectHeaderMatches{};
    // 0=none, 1=result struct, 2=GetAllValidCollisionData array,
    // 3=GetCollisionData return base, 4=boxed payload (+0x10).
    std::array<int, kNativePhysicsRouteCapacity> selectedRouteSources{};
    bool callEligible = false;
    bool callAttempted = false;
    bool available = false;
    float inputForce = 0.0f;
    float inputXSpin = 0.0f;
    float inputYSpin = 0.0f;
    float inputXMouse = 0.0f;
    float inputYMouse = 0.0f;
    int inputDegree = 0;
    int validRouteCount = 0;
    std::string status = "not_bound";
    std::array<RuntimeNativePhysicsRoute, kNativePhysicsRouteCapacity> routes{};
};

struct RuntimeSnapshot {
    bool runtimeReady = false;
    bool cameraReady = false;
    bool physicsReady = false;
    bool aimingActive = false;
    std::string status = "等待 Unity/IL2CPP";
    int unityScreenWidth = 0;
    int unityScreenHeight = 0;
    float ballRadius = 0.028575f;
    Bounds2 tableBounds{{-1.27f, -0.635f}, {1.27f, 0.635f}};
    std::string gameMode = "unknown";
    int physicsBallType = -1;
    int physicsBallCount = -1;
    bool snookerBallSet = false;
    std::string ballPositionSource = "transform";
    std::string aimSource = "none";
    int coordinateBallCount = 0;
    int activeBallCount = 0;
    Vec2 aimDirection;
    Vec2 lastAimDirection;
    Vec2 crosshairAimDirection;
    Vec2 crosshairWorld;
    bool lastAimAvailable = false;
    bool crosshairAimAvailable = false;
    float crosshairLastAngleDeltaDegrees = 0.0f;
    Vec2 gameLineScreenDirection;
    bool gameLineAvailable = false;
    float gameLineAimDeltaDegrees = 0.0f;
    std::array<RuntimePocket, 6> pockets{};
    std::array<RuntimeBall, kBallCapacity> balls{};
    RuntimePhysicsConfig physicsConfig;
    RuntimeProbe probe;
    RuntimeNativePhysicsProbe nativePhysicsProbe;
    Prediction prediction;
    PostCollisionPrediction postCollisionPrediction;
    RuntimeScreenSegment cueBeforeScreen;
    RuntimeScreenSegment cueRailBounceScreen;
    RuntimeScreenSegment cueAfterScreen;
    RuntimeScreenSegment cueAfterRailBounceScreen;
    RuntimeScreenSegment targetScreen;
    RuntimeScreenSegment targetRailBounceScreen;
    RuntimeScreenRoute cueApproachScreenRoute;
    RuntimeScreenRoute cueAfterScreenRoute;
    RuntimeScreenRoute targetScreenRoute;
    RuntimeScreenPolyline legacyCollisionScreenRoute;
    RuntimeScreenPolyline legacyCollisionExScreenRoute;
    RuntimeScreenPolyline cuePostCollisionScreenRoute;
    RuntimeScreenPolyline objectPostCollisionScreenRoute;
};

class IL2CPPBridge {
public:
    IL2CPPBridge();
    ~IL2CPPBridge();
    IL2CPPBridge(const IL2CPPBridge&) = delete;
    IL2CPPBridge& operator=(const IL2CPPBridge&) = delete;

    void setTableCalibration(float scaleX, float scaleY);
    void setPocketCalibration(float scale);
    void setBounceAngleOffset(float degrees);
    void setSecondaryBounceAngleOffset(float degrees);
    void setSecondaryBounceAngleLinked(bool linked);
    void setUseOuterRailBoundary(bool enabled);
    void setMaximumRailBounces(int count);
    void setProbeEnabled(bool enabled);
    RuntimeSnapshot sample();
    void invalidate();

private:
    struct Impl;
    Impl* impl_;
};

}  // namespace poollab
