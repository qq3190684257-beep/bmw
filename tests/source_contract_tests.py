#!/usr/bin/env python3
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
OVERLAY = (ROOT / "src" / "Overlay.mm").read_text(encoding="utf-8")
BRIDGE = (ROOT / "src" / "IL2CPPBridge.mm").read_text(encoding="utf-8")
BRIDGE_HEADER = (ROOT / "src" / "IL2CPPBridge.hpp").read_text(encoding="utf-8")
NATIVE_REFLECT = (ROOT / "src" / "NativeReflect.mm").read_text(encoding="utf-8")
MAKEFILE = (ROOT / "Makefile").read_text(encoding="utf-8")


def between(text: str, start: str, end: str) -> str:
    left = text.index(start)
    right = text.index(end, left + len(start))
    return text[left:right]


def main() -> None:
    assert 'kPoolVisibleVersion = @"0.3.0 NATIVE HYBRID"' in OVERLAY
    assert "[self buildCoreMenu]" in OVERLAY
    core_menu = between(OVERLAY, "- (void)buildCoreMenu", "- (void)buildFloatingMenu")
    for required in ("橙线：开", "开始记录", "导出日志", "_forceHudLabel"):
        assert required in core_menu
    for removed in ("角度1", "角度2", "折射次数", "球标：", "标记："):
        assert removed not in core_menu
    assert "母球坐标" not in core_menu
    assert "coordinateText" not in OVERLAY
    assert "cue_coordinates" not in OVERLAY
    assert "kBallCapacity = 32" in BRIDGE_HEADER
    assert "kBallTypeShortSnooker = 15" in BRIDGE_HEADER
    assert "kBallTypeSnooker = 16" in BRIDGE_HEADER
    assert "_snapshot.snookerBallSet" in OVERLAY
    assert "bestPocketBallUI" in BRIDGE
    assert 'snapshot.gameMode = "short_snooker_8ball_table"' in BRIDGE

    core_draw = between(
        OVERLAY,
        "// The game-native white line remains authoritative through first",
        "CGContextRestoreGState(context);",
    )
    assert "objectPostCollisionScreenRoute" in core_draw
    assert "cuePostCollisionScreenRoute" in core_draw
    assert "legacyCollisionExScreenRoute" not in core_draw
    assert "legacyCollisionScreenRoute" not in core_draw
    assert "cueApproachScreenRoute" not in core_draw
    assert "cueAfterScreenRoute" not in core_draw
    assert "targetScreenRoute" not in core_draw
    assert "getFirstCollisionSpeed" in BRIDGE
    assert "predictPostCollision" in BRIDGE
    assert "incomingSpeedFromNative" in BRIDGE
    assert "targetCollisionSpeedAvailable" in BRIDGE
    assert "post_speed_source,post_fallback_speed,post_probed_speed" in OVERLAY
    assert "post_native_cue_used,post_native_object_used" in OVERLAY
    assert "route.count < 1" in OVERLAY
    assert "launchSpeedFromForce" in BRIDGE
    assert "float xSpin = -probe.inputXSpin" in BRIDGE
    assert "static_cast<int32_t>(ballIndex)" in BRIDGE
    assert "clipPolyline(snapshot.legacyCollisionExScreenRoute)" in BRIDGE
    assert '"pocketCueReflectNum"' in NATIVE_REFLECT
    assert "value = 3" in NATIVE_REFLECT
    assert "dispatch_get_main_queue" in NATIVE_REFLECT
    assert "ScaleAim" not in NATIVE_REFLECT
    assert "shootInfo" not in NATIVE_REFLECT
    assert "src/NativeReflect.mm" in MAKEFILE
    assert "_menuButton.hidden = YES" in OVERLAY
    assert "_forceHudLabel.hidden = YES" in OVERLAY

    assert "live_force_exact,release_force_exact,force_percent,force_sample_age_ms" in OVERLAY
    collect = between(BRIDGE, "void collectProbe", "template <typename ValueT>")
    assert collect.index('readField(ui, pocketCueUIClass, "shootInfo"') < collect.index(
        "if (!probeEnabled) return;"
    )

    percent = 4.956147 / 5.05 * 100.0
    assert abs(percent - 98.1415247525) < 1.0e-6
    print("source contract ok; example percent=%.6f" % percent)


if __name__ == "__main__":
    main()
