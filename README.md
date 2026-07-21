# PoolTrajectoryHybrid 0.3.0

这是“原生翻袋 + 0.2.3 碰撞后延长”的单 dylib 混合版本。

## 显示关系

```text
游戏原生白线
  -> 母球到首碰点
  -> 游戏原生库边反射与翻袋

0.2.3 透明 Overlay
  -> 黄色实线：子球碰撞后的延长与落点
  -> 青色虚线：母球碰撞后的延长与落点
```

不再绘制 0.2.3 的橙色首碰主线，避免与游戏原生白线重叠。

## 原生翻袋设置

插件每 500 ms 从 Unity 主线程执行：

```text
GameInfo.instance
  -> Setting
  -> SettingData.pocketCueReflectNum = 3
```

每次重新读取 `GameInfo.instance` 与 `Setting`，不保存跨局场景对象地址。

## 界面

- 菜单和力度 HUD 默认隐藏。
- Overlay 窗口只负责画黄线、青线和落点，其他区域点击穿透。
- Snooker 模式仍可自动显示已发现的球标记。

## 未修改内容

- 不修改力度和出杆。
- 不修改母球或子球坐标。
- 不自动输入。
- 不修改网络数据。

## 构建与使用

把本目录内容放到 GitHub 仓库根目录，运行 Actions：

```text
Build PoolTrajectoryHybrid arm64 dylib
```

只注入生成的 `PoolTrajectoryHybrid.dylib`，不要同时注入旧的 H5GG、PocketAim、
NativeTrajectory 或 PoolTrajectoryLab dylib。
