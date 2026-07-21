#import <Foundation/Foundation.h>
#import <QuartzCore/QuartzCore.h>
#import <UIKit/UIKit.h>

#include "IL2CPPBridge.hpp"

#include <cmath>
#include <memory>

using namespace poollab;

static const CGFloat kPoolMenuPanelHeight = 148.0;
static NSString* const kPoolVisibleVersion = @"0.3.0 NATIVE HYBRID";
static const float kObservedMaximumShootForce = 5.05f;
static const CFTimeInterval kProbeSampleInterval = 1.0 / 60.0;
static const CFTimeInterval kProbeMaximumDuration = 90.0;

@interface PoolOverlayView : UIView <UITextFieldDelegate>
- (void)updateFrame:(CADisplayLink*)sender;
- (void)pauseSampling;
- (void)resumeSampling;
- (void)shutdown;
- (void)startRecording;
- (void)stopRecording;
- (void)flushPendingLogLines;
- (void)toggleRecording;
- (void)exportLog;
- (void)clearLogs;
- (void)appendProbeSampleAtTimestamp:(CFTimeInterval)timestamp;
- (void)buildCoreMenu;
@end

@interface PoolDisplayLinkProxy : NSObject
@property(nonatomic, weak) PoolOverlayView* target;
- (void)onDisplayLink:(CADisplayLink*)displayLink;
@end

@implementation PoolDisplayLinkProxy
- (void)onDisplayLink:(CADisplayLink*)displayLink {
    PoolOverlayView* target = self.target;
    [target updateFrame:displayLink];
}
@end

@interface PoolOverlayWindow : UIWindow
@end

@implementation PoolOverlayWindow

- (BOOL)canBecomeKeyWindow {
    return YES;
}

- (UIView*)hitTest:(CGPoint)point withEvent:(UIEvent*)event {
    UIView* hit = [super hitTest:point withEvent:event];
    UIView* cursor = hit;
    while (cursor) {
        if ([cursor isKindOfClass:UIControl.class])
            return hit;
        cursor = cursor.superview;
    }
    return nil;
}

@end

@implementation PoolOverlayView {
    std::unique_ptr<IL2CPPBridge> _bridge;
    RuntimeSnapshot _snapshot;
    CADisplayLink* _displayLink;
    PoolDisplayLinkProxy* _displayLinkProxy;
    UIButton* _menuButton;
    UIView* _menuPanel;
    UIButton* _predictionButton;
    UIButton* _calibrationButton;
    UIButton* _ballMarkersButton;
    UIButton* _recordButton;
    UIButton* _exportButton;
    UIButton* _clearLogButton;
    UIButton* _secondaryAngleLinkButton;
    UITextField* _primaryAngleField;
    UITextField* _secondaryAngleField;
    UITextField* _bounceCountField;
    UILabel* _railBoundaryLabel;
    UILabel* _statusLabel;
    UILabel* _logStatusLabel;
    UILabel* _geometryStatusLabel;
    UILabel* _versionLabel;
    UILabel* _forceHudLabel;
    __weak UIWindow* _previousKeyWindow;
    CGPoint _menuDragStart;
    CGFloat _menuXRatio;
    CGFloat _menuYRatio;
    BOOL _hasSavedMenuPosition;
    BOOL _menuVisible;
    BOOL _predictionEnabled;
    BOOL _calibrationEnabled;
    BOOL _ballMarkersEnabled;
    CGFloat _tableScaleX;
    CGFloat _tableScaleY;
    CGFloat _pocketScale;
    CGFloat _bounceAngleOffsetDegrees;
    CGFloat _secondaryBounceAngleOffsetDegrees;
    BOOL _secondaryBounceAngleLinked;
    BOOL _useOuterRailBoundary;
    NSInteger _maximumRailBounces;
    CFTimeInterval _fpsWindowStart;
    NSUInteger _fpsFrameCount;
    NSInteger _measuredFps;
    BOOL _samplingSuspended;
    BOOL _shuttingDown;
    BOOL _recording;
    CFTimeInterval _recordStartTimestamp;
    CFTimeInterval _lastProbeSampleTimestamp;
    NSUInteger _logRowCount;
    int _recordedTargetIndex;
    BOOL _hasPreviousAimState;
    BOOL _previousAimingActive;
    NSInteger _shotId;
    CFTimeInterval _shotStartTimestamp;
    float _liveForce;
    float _lastNonZeroForce;
    float _releaseForce;
    BOOL _hasReleaseForce;
    CFTimeInterval _lastForceSampleTimestamp;
    NSMutableArray<NSString*>* _pendingLogLines;
    NSString* _currentCSVPath;
    NSString* _currentMetadataPath;
    dispatch_queue_t _logQueue;
}

- (instancetype)initWithFrame:(CGRect)frame {
    self = [super initWithFrame:frame];
    if (!self) return nil;
    self.backgroundColor = UIColor.clearColor;
    self.opaque = NO;
    self.autoresizingMask = UIViewAutoresizingFlexibleWidth | UIViewAutoresizingFlexibleHeight;
    _bridge = std::make_unique<IL2CPPBridge>();
    _predictionEnabled = YES;
    _calibrationEnabled = NO;
    _ballMarkersEnabled = NO;
    _menuVisible = NO;
    _fpsWindowStart = 0.0;
    _fpsFrameCount = 0;
    _measuredFps = 0;
    _recording = NO;
    _recordStartTimestamp = 0.0;
    _lastProbeSampleTimestamp = 0.0;
    _logRowCount = 0;
    _recordedTargetIndex = -1;
    _hasPreviousAimState = NO;
    _previousAimingActive = NO;
    _shotId = 0;
    _shotStartTimestamp = 0.0;
    _liveForce = 0.0f;
    _lastNonZeroForce = 0.0f;
    _releaseForce = 0.0f;
    _hasReleaseForce = NO;
    _lastForceSampleTimestamp = 0.0;
    _pendingLogLines = [NSMutableArray arrayWithCapacity:120];
    _logQueue = dispatch_queue_create("com.pooltrajectorylab.probe-log", DISPATCH_QUEUE_SERIAL);
    NSUserDefaults* defaults = NSUserDefaults.standardUserDefaults;
    NSNumber* savedMenuX = [defaults objectForKey:@"PoolLabMenuXRatio"];
    NSNumber* savedMenuY = [defaults objectForKey:@"PoolLabMenuYRatio"];
    _hasSavedMenuPosition = savedMenuX && savedMenuY;
    _menuXRatio = _hasSavedMenuPosition ? savedMenuX.doubleValue : 0.0;
    _menuYRatio = _hasSavedMenuPosition ? savedMenuY.doubleValue : 0.0;
    // Core mode intentionally removes the old manual geometry controls.  The
    // EX/native route is authoritative; the geometric solver remains dormant
    // only as an internal fallback for later diagnostics.
    _bounceAngleOffsetDegrees = 0.0;
    _secondaryBounceAngleOffsetDegrees = 0.0;
    _secondaryBounceAngleLinked = YES;
    // A ball collides when its surface reaches the cushion, so its center must
    // reflect one radius inside the yellow cushion face.  The 2026-07-19 device
    // trace put the first real contact within about 0.001 world units of this
    // green ball-center boundary.
    _useOuterRailBoundary = NO;
    [defaults setBool:NO forKey:@"PoolLabUseOuterRailBoundary"];
    _maximumRailBounces = 3;
    // PocketBallUI anchors are authoritative on this build. Ignore legacy manual
    // calibration values and use the live table/pocket geometry directly.
    _tableScaleX = 1.0;
    _tableScaleY = 1.0;
    _pocketScale = 1.0;
    _bridge->setTableCalibration(static_cast<float>(_tableScaleX),
                                 static_cast<float>(_tableScaleY));
    _bridge->setPocketCalibration(static_cast<float>(_pocketScale));
    _bridge->setBounceAngleOffset(static_cast<float>(_bounceAngleOffsetDegrees));
    _bridge->setSecondaryBounceAngleOffset(
        static_cast<float>(_secondaryBounceAngleOffsetDegrees));
    _bridge->setSecondaryBounceAngleLinked(_secondaryBounceAngleLinked);
    _bridge->setUseOuterRailBoundary(_useOuterRailBoundary);
    _bridge->setMaximumRailBounces(static_cast<int>(_maximumRailBounces));

    [self buildCoreMenu];
    // Hybrid mode keeps the game's own white line as the visible primary UI.
    // The transparent overlay only contributes the post-contact extensions.
    _menuButton.hidden = YES;
    _menuPanel.hidden = YES;
    _forceHudLabel.hidden = YES;
    _bridge->setProbeEnabled(true);

    // CADisplayLink strongly owns its target. A weak proxy prevents the former
    // DisplayLink -> view -> DisplayLink retain cycle during scene teardown.
    _displayLinkProxy = [PoolDisplayLinkProxy new];
    _displayLinkProxy.target = self;
    _displayLink = [CADisplayLink displayLinkWithTarget:_displayLinkProxy
                                               selector:@selector(onDisplayLink:)];
    if (@available(iOS 15.0, *)) {
        _displayLink.preferredFrameRateRange = CAFrameRateRangeMake(60.0, 120.0, 120.0);
    } else {
        _displayLink.preferredFramesPerSecond = 60;
    }
    [_displayLink addToRunLoop:NSRunLoop.mainRunLoop forMode:NSRunLoopCommonModes];
    return self;
}

- (void)pauseSampling {
    if (_shuttingDown || _samplingSuspended) return;
    [self stopRecording];
    _samplingSuspended = YES;
    _displayLink.paused = YES;
    if (_bridge) _bridge->invalidate();
    _snapshot = RuntimeSnapshot{};
    _liveForce = 0.0f;
    _lastNonZeroForce = 0.0f;
    _releaseForce = 0.0f;
    _hasReleaseForce = NO;
    _lastForceSampleTimestamp = 0.0;
    _forceHudLabel.text = @"力度：等待球桌数据";
    _fpsWindowStart = 0.0;
    _fpsFrameCount = 0;
    [self setNeedsDisplay];
}

- (void)resumeSampling {
    if (_shuttingDown || !_displayLink) return;
    if (UIApplication.sharedApplication.applicationState != UIApplicationStateActive) return;
    // Always resolve classes and scene objects again after activation. Never reuse
    // Camera/CueUI/Transform pointers obtained from the previous Unity scene.
    if (_bridge) _bridge->invalidate();
    _samplingSuspended = NO;
    _fpsWindowStart = 0.0;
    _fpsFrameCount = 0;
    _displayLink.paused = NO;
}

- (void)shutdown {
    if (_shuttingDown) return;
    [self stopRecording];
    _shuttingDown = YES;
    _samplingSuspended = YES;
    [_displayLink invalidate];
    _displayLink = nil;
    _displayLinkProxy.target = nil;
    _displayLinkProxy = nil;
    if (_bridge) {
        _bridge->invalidate();
        _bridge.reset();
    }
    _snapshot = RuntimeSnapshot{};
}

- (UIButton*)makeButton:(NSString*)title frame:(CGRect)frame parent:(UIView*)parent action:(SEL)action {
    UIButton* button = [UIButton buttonWithType:UIButtonTypeSystem];
    button.frame = frame;
    button.backgroundColor = [UIColor colorWithWhite:0.05 alpha:0.72];
    button.layer.cornerRadius = 8.0;
    button.layer.borderColor = [UIColor colorWithRed:0.1 green:0.8 blue:1.0 alpha:0.9].CGColor;
    button.layer.borderWidth = 1.0;
    [button setTitle:title forState:UIControlStateNormal];
    [button setTitleColor:UIColor.whiteColor forState:UIControlStateNormal];
    button.titleLabel.font = [UIFont systemFontOfSize:13.0 weight:UIFontWeightSemibold];
    [button addTarget:self action:action forControlEvents:UIControlEventTouchUpInside];
    [parent addSubview:button];
    return button;
}

- (UILabel*)makeLabel:(NSString*)text frame:(CGRect)frame fontSize:(CGFloat)fontSize {
    UILabel* label = [[UILabel alloc] initWithFrame:frame];
    label.text = text;
    label.textColor = UIColor.whiteColor;
    label.font = [UIFont systemFontOfSize:fontSize weight:UIFontWeightMedium];
    return label;
}

- (UITextField*)makeNumberField:(CGRect)frame parent:(UIView*)parent {
    UITextField* field = [[UITextField alloc] initWithFrame:frame];
    field.backgroundColor = [UIColor colorWithWhite:0.05 alpha:0.78];
    field.layer.cornerRadius = 8.0;
    field.layer.borderColor =
        [UIColor colorWithRed:0.1 green:0.8 blue:1.0 alpha:0.9].CGColor;
    field.layer.borderWidth = 1.0;
    field.textColor = UIColor.whiteColor;
    field.tintColor = [UIColor colorWithRed:0.15 green:0.9 blue:1.0 alpha:1.0];
    field.textAlignment = NSTextAlignmentCenter;
    field.font = [UIFont monospacedDigitSystemFontOfSize:13.0
                                                  weight:UIFontWeightSemibold];
    field.keyboardType = UIKeyboardTypeNumbersAndPunctuation;
    field.returnKeyType = UIReturnKeyDone;
    field.autocorrectionType = UITextAutocorrectionTypeNo;
    field.spellCheckingType = UITextSpellCheckingTypeNo;
    field.delegate = self;

    UIToolbar* toolbar = [[UIToolbar alloc] initWithFrame:
        CGRectMake(0.0, 0.0, CGRectGetWidth(self.bounds), 44.0)];
    UIBarButtonItem* cancel = [[UIBarButtonItem alloc]
        initWithTitle:@"取消" style:UIBarButtonItemStylePlain
        target:self action:@selector(cancelNumericEntry)];
    UIBarButtonItem* flexible = [[UIBarButtonItem alloc]
        initWithBarButtonSystemItem:UIBarButtonSystemItemFlexibleSpace
        target:nil action:nil];
    UIBarButtonItem* done = [[UIBarButtonItem alloc]
        initWithTitle:@"确定" style:UIBarButtonItemStyleDone
        target:self action:@selector(finishNumericEntry)];
    toolbar.items = @[cancel, flexible, done];
    field.inputAccessoryView = toolbar;
    [parent addSubview:field];
    return field;
}

- (void)buildCoreMenu {
    _menuButton = [self makeButton:@"菜单" frame:CGRectMake(10.0, 10.0, 58.0, 38.0)
                            parent:self action:@selector(toggleMenu)];
    _menuButton.backgroundColor =
        [UIColor colorWithRed:0.02 green:0.18 blue:0.25 alpha:0.88];
    UIPanGestureRecognizer* drag = [[UIPanGestureRecognizer alloc]
        initWithTarget:self action:@selector(dragMenu:)];
    drag.maximumNumberOfTouches = 1;
    [_menuButton addGestureRecognizer:drag];

    _menuPanel = [[UIView alloc] initWithFrame:
        CGRectMake(10.0, 54.0, 350.0, kPoolMenuPanelHeight)];
    _menuPanel.backgroundColor = [UIColor colorWithWhite:0.03 alpha:0.88];
    _menuPanel.layer.cornerRadius = 10.0;
    _menuPanel.layer.borderWidth = 1.0;
    _menuPanel.layer.borderColor =
        [UIColor colorWithRed:0.1 green:0.8 blue:1.0 alpha:0.9].CGColor;
    _menuPanel.hidden = YES;
    [self addSubview:_menuPanel];

    _predictionButton = [self makeButton:@"橙线：开"
        frame:CGRectMake(10.0, 10.0, 105.0, 34.0)
        parent:_menuPanel action:@selector(togglePrediction)];
    _recordButton = [self makeButton:@"开始记录"
        frame:CGRectMake(122.0, 10.0, 105.0, 34.0)
        parent:_menuPanel action:@selector(toggleRecording)];
    _exportButton = [self makeButton:@"导出日志"
        frame:CGRectMake(234.0, 10.0, 105.0, 34.0)
        parent:_menuPanel action:@selector(exportLog)];

    _statusLabel = [self makeLabel:@"等待球桌数据"
        frame:CGRectMake(12.0, 50.0, 326.0, 21.0) fontSize:11.0];
    _statusLabel.textColor =
        [UIColor colorWithRed:0.45 green:0.95 blue:1.0 alpha:1.0];
    _statusLabel.adjustsFontSizeToFitWidth = YES;
    _statusLabel.minimumScaleFactor = 0.72;
    [_menuPanel addSubview:_statusLabel];

    _logStatusLabel = [self makeLabel:@"日志：未记录"
        frame:CGRectMake(12.0, 74.0, 326.0, 20.0) fontSize:11.0];
    _logStatusLabel.textColor =
        [UIColor colorWithRed:1.0 green:0.82 blue:0.35 alpha:1.0];
    _logStatusLabel.adjustsFontSizeToFitWidth = YES;
    _logStatusLabel.minimumScaleFactor = 0.7;
    [_menuPanel addSubview:_logStatusLabel];

    UILabel* coreLabel = [self makeLabel:@"EX橙线 / 子球黄线 / 母球青线 / 精确力度"
        frame:CGRectMake(12.0, 99.0, 326.0, 20.0) fontSize:10.5];
    coreLabel.textAlignment = NSTextAlignmentCenter;
    coreLabel.textColor = [UIColor colorWithRed:0.55 green:1.0 blue:0.62 alpha:1.0];
    coreLabel.adjustsFontSizeToFitWidth = YES;
    coreLabel.minimumScaleFactor = 0.72;
    [_menuPanel addSubview:coreLabel];

    _versionLabel = [self makeLabel:kPoolVisibleVersion
        frame:CGRectMake(12.0, 123.0, 326.0, 16.0) fontSize:9.5];
    _versionLabel.textAlignment = NSTextAlignmentCenter;
    _versionLabel.textColor = [UIColor colorWithWhite:0.72 alpha:1.0];
    [_menuPanel addSubview:_versionLabel];

    _forceHudLabel = [self makeLabel:@"力度：等待球桌数据"
        frame:CGRectMake(0.0, 0.0, 344.0, 30.0) fontSize:12.0];
    _forceHudLabel.textAlignment = NSTextAlignmentCenter;
    _forceHudLabel.font = [UIFont monospacedDigitSystemFontOfSize:12.0
                                                           weight:UIFontWeightSemibold];
    _forceHudLabel.backgroundColor = [UIColor colorWithWhite:0.02 alpha:0.76];
    _forceHudLabel.layer.cornerRadius = 7.0;
    _forceHudLabel.layer.borderWidth = 1.0;
    _forceHudLabel.layer.borderColor =
        [UIColor colorWithRed:1.0 green:0.45 blue:0.05 alpha:0.95].CGColor;
    _forceHudLabel.clipsToBounds = YES;
    [self addSubview:_forceHudLabel];
}

- (void)buildFloatingMenu {
    _menuButton = [self makeButton:@"菜单" frame:CGRectMake(10.0, 10.0, 58.0, 38.0)
                            parent:self action:@selector(toggleMenu)];
    _menuButton.backgroundColor = [UIColor colorWithRed:0.02 green:0.18 blue:0.25 alpha:0.88];
    UIPanGestureRecognizer* drag = [[UIPanGestureRecognizer alloc] initWithTarget:self
                                                                          action:@selector(dragMenu:)];
    drag.maximumNumberOfTouches = 1;
    [_menuButton addGestureRecognizer:drag];

    _menuPanel = [[UIView alloc] initWithFrame:CGRectMake(10.0, 54.0, 350.0,
                                                         kPoolMenuPanelHeight)];
    _menuPanel.backgroundColor = [UIColor colorWithWhite:0.03 alpha:0.88];
    _menuPanel.layer.cornerRadius = 10.0;
    _menuPanel.layer.borderWidth = 1.0;
    _menuPanel.layer.borderColor = [UIColor colorWithRed:0.1 green:0.8 blue:1.0 alpha:0.9].CGColor;
    _menuPanel.hidden = YES;
    [self addSubview:_menuPanel];

    _predictionButton = [self makeButton:@"预测：关" frame:CGRectMake(10.0, 10.0, 100.0, 34.0)
                                   parent:_menuPanel action:@selector(togglePrediction)];
    _calibrationButton = [self makeButton:@"标记：关" frame:CGRectMake(125.0, 10.0, 100.0, 34.0)
                                    parent:_menuPanel action:@selector(toggleCalibration)];
    _ballMarkersButton = [self makeButton:@"球标：关" frame:CGRectMake(240.0, 10.0, 100.0, 34.0)
                                    parent:_menuPanel action:@selector(toggleBallMarkers)];

    _statusLabel = [self makeLabel:@"状态：等待运行时数据"
                              frame:CGRectMake(12.0, 50.0, 328.0, 23.0) fontSize:11.0];
    _statusLabel.textColor = [UIColor colorWithRed:0.45 green:0.95 blue:1.0 alpha:1.0];
    _statusLabel.adjustsFontSizeToFitWidth = YES;
    _statusLabel.minimumScaleFactor = 0.75;
    [_menuPanel addSubview:_statusLabel];

    _recordButton = [self makeButton:@"开始记录" frame:CGRectMake(10.0, 78.0, 105.0, 34.0)
                              parent:_menuPanel action:@selector(toggleRecording)];
    _exportButton = [self makeButton:@"导出日志" frame:CGRectMake(122.0, 78.0, 105.0, 34.0)
                              parent:_menuPanel action:@selector(exportLog)];
    _clearLogButton = [self makeButton:@"清空日志" frame:CGRectMake(234.0, 78.0, 105.0, 34.0)
                                parent:_menuPanel action:@selector(clearLogs)];
    _logStatusLabel = [self makeLabel:@"日志：未记录"
                                frame:CGRectMake(12.0, 116.0, 326.0, 20.0) fontSize:11.0];
    _logStatusLabel.textColor = [UIColor colorWithRed:1.0 green:0.82 blue:0.35 alpha:1.0];
    _logStatusLabel.adjustsFontSizeToFitWidth = YES;
    _logStatusLabel.minimumScaleFactor = 0.7;
    [_menuPanel addSubview:_logStatusLabel];

    UILabel* angleLabel = [self makeLabel:@"角度1"
                                    frame:CGRectMake(12.0, 143.0, 48.0, 32.0) fontSize:11.0];
    [_menuPanel addSubview:angleLabel];
    _primaryAngleField = [self makeNumberField:CGRectMake(66.0, 143.0, 120.0, 32.0)
                                         parent:_menuPanel];
    _primaryAngleField.placeholder = @"-30.00 ~ 30.00";
    _railBoundaryLabel = [self makeLabel:@"碰库：绿色球心线"
                                   frame:CGRectMake(194.0, 143.0, 145.0, 32.0) fontSize:11.0];
    _railBoundaryLabel.textAlignment = NSTextAlignmentCenter;
    _railBoundaryLabel.textColor = [UIColor colorWithRed:0.35 green:1.0 blue:0.48 alpha:1.0];
    [_menuPanel addSubview:_railBoundaryLabel];

    UILabel* secondAngleLabel = [self makeLabel:@"角度2"
                                          frame:CGRectMake(12.0, 180.0, 48.0, 32.0)
                                       fontSize:11.0];
    [_menuPanel addSubview:secondAngleLabel];
    _secondaryAngleField = [self makeNumberField:CGRectMake(66.0, 180.0, 120.0, 32.0)
                                           parent:_menuPanel];
    _secondaryAngleField.placeholder = @"-30.00 ~ 30.00";
    _secondaryAngleLinkButton = [self makeButton:
        (_secondaryBounceAngleLinked ? @"二角联动：开" : @"二角联动：关")
                                                  frame:CGRectMake(194.0, 180.0, 145.0, 32.0)
                                                 parent:_menuPanel
                                                 action:@selector(toggleSecondaryAngleLink)];

    UILabel* countLabel = [self makeLabel:@"折射次数"
                                    frame:CGRectMake(12.0, 217.0, 70.0, 32.0) fontSize:11.0];
    [_menuPanel addSubview:countLabel];
    _bounceCountField = [self makeNumberField:CGRectMake(86.0, 217.0, 100.0, 32.0)
                                        parent:_menuPanel];
    _bounceCountField.placeholder = @"1 ~ 6";
    UILabel* stepLabel = [self makeLabel:@"点击输入，确定后生效"
                                   frame:CGRectMake(194.0, 217.0, 145.0, 32.0) fontSize:11.0];
    stepLabel.textAlignment = NSTextAlignmentCenter;
    [_menuPanel addSubview:stepLabel];

    _geometryStatusLabel = [self makeLabel:@"长 -- 宽 -- / 角1 +0.00° 角2 +0.00°联 / 库1"
                                      frame:CGRectMake(12.0, 250.0, 326.0, 20.0) fontSize:10.5];
    _geometryStatusLabel.textColor = [UIColor colorWithRed:0.55 green:1.0 blue:0.62 alpha:1.0];
    _geometryStatusLabel.adjustsFontSizeToFitWidth = YES;
    _geometryStatusLabel.minimumScaleFactor = 0.68;
    [_menuPanel addSubview:_geometryStatusLabel];

    _versionLabel = [self makeLabel:
        [NSString stringWithFormat:@"%@ / V0 white / EX orange", kPoolVisibleVersion]
                                  frame:CGRectMake(12.0, 271.0, 326.0, 14.0)
                               fontSize:9.5];
    _versionLabel.textAlignment = NSTextAlignmentCenter;
    _versionLabel.textColor = [UIColor colorWithWhite:0.72 alpha:1.0];
    _versionLabel.adjustsFontSizeToFitWidth = YES;
    _versionLabel.minimumScaleFactor = 0.68;
    [_menuPanel addSubview:_versionLabel];
    [self refreshTrajectoryControls];
}

- (void)layoutSubviews {
    [super layoutSubviews];
    UIEdgeInsets safe = self.safeAreaInsets;
    CGFloat minX = MAX(10.0, safe.left + 6.0);
    CGFloat minY = MAX(10.0, safe.top + 6.0);
    CGFloat maxX = MAX(minX, CGRectGetWidth(self.bounds) - safe.right - 64.0);
    CGFloat maxY = MAX(minY, CGRectGetHeight(self.bounds) - safe.bottom - 44.0);
    CGFloat x = _hasSavedMenuPosition ? minX + (maxX - minX) * MAX(0.0, MIN(1.0, _menuXRatio)) : minX;
    CGFloat y = _hasSavedMenuPosition ? minY + (maxY - minY) * MAX(0.0, MIN(1.0, _menuYRatio)) : minY;
    _menuButton.frame = CGRectMake(x, y, 58.0, 38.0);
    const CGFloat forceWidth = MIN(360.0, CGRectGetWidth(self.bounds) -
        safe.left - safe.right - 24.0);
    _forceHudLabel.frame = CGRectMake(
        (CGRectGetWidth(self.bounds) - forceWidth) * 0.5,
        safe.top + 6.0, forceWidth, 30.0);
    [self positionMenuPanel];
}

- (void)positionMenuPanel {
    UIEdgeInsets safe = self.safeAreaInsets;
    CGFloat panelX = MIN(CGRectGetMinX(_menuButton.frame),
                         CGRectGetWidth(self.bounds) - safe.right - 356.0);
    panelX = MAX(safe.left + 6.0, panelX);
    CGFloat panelY = CGRectGetMaxY(_menuButton.frame) + 6.0;
    if (panelY + kPoolMenuPanelHeight > CGRectGetHeight(self.bounds) - safe.bottom - 6.0)
        panelY = CGRectGetMinY(_menuButton.frame) - kPoolMenuPanelHeight - 6.0;
    panelY = MAX(safe.top + 6.0, panelY);
    _menuPanel.frame = CGRectMake(panelX, panelY, 350.0, kPoolMenuPanelHeight);
}

- (void)dragMenu:(UIPanGestureRecognizer*)gesture {
    if (gesture.state == UIGestureRecognizerStateBegan)
        _menuDragStart = _menuButton.frame.origin;
    CGPoint translation = [gesture translationInView:self];
    UIEdgeInsets safe = self.safeAreaInsets;
    CGFloat minX = MAX(10.0, safe.left + 6.0);
    CGFloat minY = MAX(10.0, safe.top + 6.0);
    CGFloat maxX = MAX(minX, CGRectGetWidth(self.bounds) - safe.right - 64.0);
    CGFloat maxY = MAX(minY, CGRectGetHeight(self.bounds) - safe.bottom - 44.0);
    CGFloat x = MAX(minX, MIN(maxX, _menuDragStart.x + translation.x));
    CGFloat y = MAX(minY, MIN(maxY, _menuDragStart.y + translation.y));
    _menuButton.frame = CGRectMake(x, y, 58.0, 38.0);
    [self positionMenuPanel];

    if (gesture.state == UIGestureRecognizerStateEnded ||
        gesture.state == UIGestureRecognizerStateCancelled) {
        _menuXRatio = maxX > minX ? (x - minX) / (maxX - minX) : 0.0;
        _menuYRatio = maxY > minY ? (y - minY) / (maxY - minY) : 0.0;
        _hasSavedMenuPosition = YES;
        NSUserDefaults* defaults = NSUserDefaults.standardUserDefaults;
        [defaults setDouble:_menuXRatio forKey:@"PoolLabMenuXRatio"];
        [defaults setDouble:_menuYRatio forKey:@"PoolLabMenuYRatio"];
    }
}

- (void)toggleMenu {
    _menuVisible = !_menuVisible;
    _menuPanel.hidden = !_menuVisible;
    [_menuButton setTitle:_menuVisible ? @"收起" : @"菜单" forState:UIControlStateNormal];
    if (_menuVisible) {
        for (UIWindow* candidate in self.window.windowScene.windows) {
            if (candidate.isKeyWindow && candidate != self.window) {
                _previousKeyWindow = candidate;
                break;
            }
        }
        [self.window makeKeyWindow];
    } else {
        [self endEditing:YES];
        [self restoreGameKeyWindow];
    }
}

- (void)restoreGameKeyWindow {
    UIWindow* previous = _previousKeyWindow;
    if (previous && !previous.hidden) {
        [previous makeKeyWindow];
        return;
    }
    for (UIWindow* candidate in self.window.windowScene.windows) {
        if (candidate == self.window || candidate.hidden || candidate.alpha <= 0.0) continue;
        if (candidate.windowLevel == UIWindowLevelNormal) {
            [candidate makeKeyWindow];
            return;
        }
    }
}

- (void)togglePrediction {
    _predictionEnabled = !_predictionEnabled;
    [_predictionButton setTitle:_predictionEnabled ? @"橙线：开" : @"橙线：关"
                        forState:UIControlStateNormal];
    if (_bridge)
        _bridge->setProbeEnabled(_recording || _predictionEnabled);
    [self setNeedsDisplay];
}

- (void)toggleCalibration {
    _calibrationEnabled = !_calibrationEnabled;
    [_calibrationButton setTitle:_calibrationEnabled ? @"标记：开" : @"标记：关"
                         forState:UIControlStateNormal];
    if (_bridge) _bridge->setProbeEnabled(_recording || _calibrationEnabled ||
                                          _predictionEnabled);
    [self setNeedsDisplay];
}

- (void)toggleBallMarkers {
    _ballMarkersEnabled = !_ballMarkersEnabled;
    [_ballMarkersButton setTitle:_ballMarkersEnabled ? @"球标：开" : @"球标：关"
                         forState:UIControlStateNormal];
    [self setNeedsDisplay];
}

- (void)refreshTrajectoryControls {
    _primaryAngleField.text = [NSString stringWithFormat:@"%+.2f",
                                                         _bounceAngleOffsetDegrees];
    _secondaryAngleField.text = [NSString stringWithFormat:@"%+.2f",
                                                           _secondaryBounceAngleOffsetDegrees];
    _bounceCountField.text = [NSString stringWithFormat:@"%ld",
                                                        (long)_maximumRailBounces];
    const BOOL secondaryEnabled = !_secondaryBounceAngleLinked;
    _secondaryAngleField.enabled = secondaryEnabled;
    _secondaryAngleField.alpha = secondaryEnabled ? 1.0 : 0.45;
    [_secondaryAngleLinkButton setTitle:
        (_secondaryBounceAngleLinked ? @"二角联动：开" : @"二角联动：关")
                                  forState:UIControlStateNormal];
}

- (BOOL)scanNumberFromField:(UITextField*)field value:(double*)value {
    NSString* rawInput = field.text ?: @"";
    NSString* input = [rawInput
        stringByTrimmingCharactersInSet:NSCharacterSet.whitespaceAndNewlineCharacterSet];
    input = [input stringByReplacingOccurrencesOfString:@"," withString:@"."];
    if (input.length == 0) return NO;
    NSScanner* scanner = [NSScanner scannerWithString:input];
    double scanned = 0.0;
    if (![scanner scanDouble:&scanned] || scanner.scanLocation != input.length ||
        !std::isfinite(scanned)) return NO;
    if (value) *value = scanned;
    return YES;
}

- (void)applyTrajectoryInputFields {
    double value = 0.0;
    if ([self scanNumberFromField:_primaryAngleField value:&value]) {
        _bounceAngleOffsetDegrees = clampedBounceAngleOffset(
            static_cast<float>(value));
    }
    if (_secondaryBounceAngleLinked) {
        _secondaryBounceAngleOffsetDegrees = _bounceAngleOffsetDegrees;
    } else if ([self scanNumberFromField:_secondaryAngleField value:&value]) {
        _secondaryBounceAngleOffsetDegrees = clampedBounceAngleOffset(
            static_cast<float>(value));
    }
    if ([self scanNumberFromField:_bounceCountField value:&value]) {
        const NSInteger rounded = static_cast<NSInteger>(std::lround(value));
        if (std::fabs(value - rounded) <= 1.0e-3)
            _maximumRailBounces = clampedRailBounceCount(static_cast<int>(rounded));
    }
    [self persistAndApplyTrajectorySettings];
}

- (void)finishNumericEntry {
    [self applyTrajectoryInputFields];
    [self endEditing:YES];
}

- (void)cancelNumericEntry {
    [self refreshTrajectoryControls];
    [self endEditing:YES];
}

- (BOOL)textFieldShouldReturn:(UITextField*)textField {
    (void)textField;
    [self finishNumericEntry];
    return NO;
}

- (void)textFieldDidBeginEditing:(UITextField*)textField {
    dispatch_async(dispatch_get_main_queue(), ^{
        [textField selectAll:nil];
    });
}

- (void)persistAndApplyTrajectorySettings {
    NSUserDefaults* defaults = NSUserDefaults.standardUserDefaults;
    [defaults setDouble:_bounceAngleOffsetDegrees forKey:@"PoolLabBounceAngleDegrees"];
    [defaults setDouble:_secondaryBounceAngleOffsetDegrees
                  forKey:@"PoolLabSecondaryBounceAngleDegrees"];
    [defaults setBool:_secondaryBounceAngleLinked
                forKey:@"PoolLabSecondaryBounceAngleLinked"];
    [defaults setInteger:_maximumRailBounces forKey:@"PoolLabMaximumRailBounces"];
    [defaults setBool:_useOuterRailBoundary forKey:@"PoolLabUseOuterRailBoundary"];
    if (_bridge) {
        _bridge->setBounceAngleOffset(static_cast<float>(_bounceAngleOffsetDegrees));
        _bridge->setSecondaryBounceAngleOffset(
            static_cast<float>(_secondaryBounceAngleOffsetDegrees));
        _bridge->setSecondaryBounceAngleLinked(_secondaryBounceAngleLinked);
        _bridge->setUseOuterRailBoundary(_useOuterRailBoundary);
        _bridge->setMaximumRailBounces(static_cast<int>(_maximumRailBounces));
    }
    [self refreshTrajectoryControls];
    [self setNeedsDisplay];
}

- (void)toggleSecondaryAngleLink {
    [self applyTrajectoryInputFields];
    _secondaryBounceAngleLinked = !_secondaryBounceAngleLinked;
    if (_secondaryBounceAngleLinked)
        _secondaryBounceAngleOffsetDegrees = _bounceAngleOffsetDegrees;
    [self persistAndApplyTrajectorySettings];
}

- (NSString*)probeLogDirectory {
    NSString* documents = NSSearchPathForDirectoriesInDomains(NSDocumentDirectory,
                                                               NSUserDomainMask, YES).firstObject;
    return [[documents stringByAppendingPathComponent:@"PoolTrajectoryHybrid"]
            stringByAppendingPathComponent:@"Logs"];
}

- (NSString*)probeTimestampName {
    NSDateFormatter* formatter = [NSDateFormatter new];
    formatter.locale = [NSLocale localeWithLocaleIdentifier:@"en_US_POSIX"];
    formatter.timeZone = [NSTimeZone localTimeZone];
    formatter.dateFormat = @"yyyyMMdd-HHmmss";
    return [formatter stringFromDate:[NSDate date]];
}

- (void)startRecording {
    if (_recording || !_bridge) return;
    NSString* directory = [self probeLogDirectory];
    NSError* error = nil;
    if (![NSFileManager.defaultManager createDirectoryAtPath:directory
                                withIntermediateDirectories:YES attributes:nil error:&error]) {
        _logStatusLabel.text = [NSString stringWithFormat:@"日志：创建目录失败 %@",
                                                          error.localizedDescription ?: @""];
        return;
    }

    NSString* base = [NSString stringWithFormat:@"probe-%@", [self probeTimestampName]];
    _currentCSVPath = [directory stringByAppendingPathComponent:[base stringByAppendingString:@".csv"]];
    _currentMetadataPath = [directory stringByAppendingPathComponent:[base stringByAppendingString:@".txt"]];
    NSString* header = @"t_ms,runtime_ready,probe_available,is_show_line,aiming_active,"
                        @"force_value,force_duration,shot_force,added_force,shoot_value,shoot_force,"
                        @"x_spin,y_spin,x_mouse,y_mouse,degree,rand_ic,trace,line_count,line_points,"
                        @"x_count,x_values,y_count,y_values,cue_visible,cue_x,cue_y,target_index,"
                        @"target_visible,target_x,target_y,physics_model_found,physics_available,"
                        @"physics_edge_count,physics_edges,physics_hole_count,physics_holes,"
                        @"coord_width,coord_height,coord_scale,coord_offset_x,coord_offset_y,"
                         @"ball_screen_radius,bounds_source,table_min_x,table_min_y,table_max_x,table_max_y,"
                         @"rail_min_x,rail_min_y,rail_max_x,rail_max_y,"
                         @"game_mode,ball_source,coordinate_ball_count,active_ball_count,ball_map,"
                         @"aim_source,aim_x,aim_y,last_dir_valid,last_dir_x,last_dir_y,"
                         @"crosshair_valid,crosshair_dir_x,crosshair_dir_y,crosshair_world_x,crosshair_world_y,"
                         @"crosshair_last_delta_deg,game_line_valid,game_line_dx,game_line_dy,"
                         @"game_line_aim_delta_deg,shot_id,shot_phase,shot_elapsed_ms,"
                         @"angle_1_deg,angle_2_deg,angle_2_linked,max_rail_bounces,rail_boundary,"
                         @"cue_route,cue_after_route,target_route,"
                         @"native_class_found,native_method_found,native_call_eligible,"
                         @"native_state_methods_found,native_state_attempted,native_state_available,"
                         @"native_engine_ball_count,native_engine_ball_type,native_engine_position_count,"
                         @"native_transform_comparison_count,native_transform_max_delta,native_engine_balls,"
                         @"native_mode_fields_found,native_mode_values_available,native_use_ex,"
                         @"native_use_ex_v0,native_use_ex_v1,native_both_mode,"
                         @"legacy_method_found,legacy_call_attempted,legacy_sret_changed_bytes,"
                         @"legacy_requested_count,legacy_reported_count,legacy_scanned_count,"
                         @"legacy_captured_count,legacy_collision_ball,legacy_max_chord_residual,"
                         @"legacy_valid,legacy_status,legacy_points,"
                         @"legacy_ex_method_found,legacy_ex_call_attempted,legacy_ex_sret_changed_bytes,"
                         @"legacy_ex_requested_count,legacy_ex_reported_count,legacy_ex_scanned_count,"
                         @"legacy_ex_captured_count,legacy_ex_collision_ball,legacy_ex_max_chord_residual,"
                         @"legacy_ex_valid,legacy_ex_status,legacy_ex_points,"
                         @"native_managed_parser_found,native_managed_valid_count_available,"
                         @"native_managed_valid_count,native_value_box_available,"
                         @"native_managed_array_method_found,native_managed_array_available,"
                         @"native_managed_array_length,native_direct_sret_used,"
                         @"native_direct_sret_changed_bytes,native_direct_collision_valid_count,"
                         @"native_direct_valid_count,"
                         @"native_raw_routes,native_layout_routes,"
                         @"native_selected_sources,"
                         @"native_call_attempted,native_available,native_status,native_force,"
                         @"native_x_spin,native_y_spin,native_effective_x_spin,native_effective_y_spin,"
                         @"native_x_mouse,native_y_mouse,native_degree,"
                         @"native_route_count,native_routes,native_line_methods_found,"
                         @"native_lines_available,native_edge_line_count,native_edge_lines,"
                         @"native_hole_line_count,native_hole_lines,"
                         @"live_force_exact,release_force_exact,force_percent,force_sample_age_ms,"
                         @"first_speed_method_found,first_speed_attempted,first_speed_available,"
                         @"first_speed_raw_x,first_speed_raw_y,first_speed_world_x,first_speed_world_y,"
                         @"first_speed_ball_index,target_speed_attempted,target_speed_available,"
                         @"target_speed_ball_index,target_speed_raw_x,target_speed_raw_y,"
                         @"target_speed_world_x,target_speed_world_y,"
                         @"post_prediction_valid,post_target_index,post_impact_x,post_impact_y,"
                         @"post_incoming_speed,post_speed_source,post_fallback_speed,post_probed_speed,"
                         @"post_native_cue_used,post_native_object_used,"
                         @"post_object_collisions,post_object_rails,"
                         @"cue_post_distance,object_post_distance,"
                         @"cue_stop_x,cue_stop_y,object_stop_x,object_stop_y,"
                         @"cue_post_route,object_post_route\n";
    if (![header writeToFile:_currentCSVPath atomically:YES
                    encoding:NSUTF8StringEncoding error:&error]) {
        _logStatusLabel.text = [NSString stringWithFormat:@"日志：创建文件失败 %@",
                                                          error.localizedDescription ?: @""];
        _currentCSVPath = nil;
        _currentMetadataPath = nil;
        return;
    }

    NSBundle* bundle = NSBundle.mainBundle;
    NSString* gameVersion = [bundle objectForInfoDictionaryKey:@"CFBundleShortVersionString"] ?: @"unknown";
    NSString* metadata = [NSString stringWithFormat:
        @"plugin_version=0.3.0-native-hybrid\n"
         @"game_bundle=%@\n"
         @"game_version=%@\n"
         @"device=%@\n"
         @"system=%@ %@\n"
         @"detected_mode=%s\n"
         @"coordinate_ball_count=%d\n"
         @"ball_position_source=%s\n"
         @"angle_1_degrees=%.2f\n"
         @"angle_2_degrees=%.2f\n"
         @"angle_2_linked=%d\n"
         @"rail_boundary=%@\n"
         @"native_physics_method=PhysicsEx.PhysicsCall.getAllCollisionData/getAllCollisionDataEx/getAllCollisionDataFinalSimple\n"
         @"native_physics_policy=read_only_ex_route_first_collision_speed_10hz_force_capture_60hz\n"
         @"visible_features=game_native_white_object_yellow_cue_cyan_landing_snooker_balls\n"
         @"primary_route=game_native_white_line\n"
         @"duplicate_legacy_ex_route=hidden\n"
         @"post_collision_model=slide_3.672_roll_0.1163_ball_e_0.734\n"
         @"force_distance_formula=piecewise_linear_calibration_slide_to_5over7_then_roll\n"
         @"force_ui_policy=ignore_rounded_game_text_use_shootInfo_fForce\n"
         @"maximum_force_internal_observed=5.05\n"
         @"sample_rate_hz=60\n"
         @"maximum_duration_seconds=90\n",
         bundle.bundleIdentifier ?: @"unknown", gameVersion, UIDevice.currentDevice.model,
         UIDevice.currentDevice.systemName, UIDevice.currentDevice.systemVersion,
         _snapshot.gameMode.c_str(), _snapshot.coordinateBallCount,
         _snapshot.ballPositionSource.c_str(), _bounceAngleOffsetDegrees,
         _secondaryBounceAngleOffsetDegrees, _secondaryBounceAngleLinked ? 1 : 0,
         _useOuterRailBoundary ? @"dashed_outer" : @"ball_center"];
    [metadata writeToFile:_currentMetadataPath atomically:YES
                 encoding:NSUTF8StringEncoding error:nil];

    [_pendingLogLines removeAllObjects];
    _logRowCount = 0;
    _recordedTargetIndex = -1;
    _hasPreviousAimState = NO;
    _previousAimingActive = NO;
    _shotId = 0;
    _shotStartTimestamp = 0.0;
    _releaseForce = 0.0f;
    _hasReleaseForce = NO;
    _recordStartTimestamp = 0.0;
    _lastProbeSampleTimestamp = 0.0;
    _recording = YES;
    _bridge->setProbeEnabled(true);
    [_recordButton setTitle:@"停止记录" forState:UIControlStateNormal];
    _recordButton.backgroundColor = [UIColor colorWithRed:0.55 green:0.08 blue:0.08 alpha:0.88];
    _logStatusLabel.text = @"日志：记录中 0 条";
}

- (void)flushPendingLogLines {
    if (_pendingLogLines.count == 0 || _currentCSVPath.length == 0) return;
    NSArray<NSString*>* lines = [_pendingLogLines copy];
    [_pendingLogLines removeAllObjects];
    NSString* path = [_currentCSVPath copy];
    dispatch_async(_logQueue, ^{
        @autoreleasepool {
            NSString* chunk = [lines componentsJoinedByString:@""];
            NSData* data = [chunk dataUsingEncoding:NSUTF8StringEncoding];
            NSFileHandle* file = [NSFileHandle fileHandleForWritingAtPath:path];
            if (!file || !data) return;
            @try {
                [file seekToEndOfFile];
                [file writeData:data];
                [file synchronizeFile];
            } @catch (__unused NSException* exception) {
            }
            [file closeFile];
        }
    });
}

- (void)stopRecording {
    if (!_recording) return;
    _recording = NO;
    if (_bridge) _bridge->setProbeEnabled(_calibrationEnabled || _predictionEnabled);
    [self flushPendingLogLines];
    [_recordButton setTitle:@"开始记录" forState:UIControlStateNormal];
    _recordButton.backgroundColor = [UIColor colorWithWhite:0.05 alpha:0.72];
    _logStatusLabel.text = [NSString stringWithFormat:@"日志：已停止 %lu 条",
                                                      (unsigned long)_logRowCount];
}

- (void)toggleRecording {
    if (_recording) [self stopRecording];
    else [self startRecording];
}

- (NSString*)probeLinePoints:(const RuntimeProbe&)probe {
    NSMutableString* text = [NSMutableString string];
    for (int i = 0; i < probe.lineDataCaptured; ++i) {
        if (i) [text appendString:@"|"];
        const Vec2 point = probe.lineData[static_cast<std::size_t>(i)];
        [text appendFormat:@"%.6g:%.6g", point.x, point.y];
    }
    return text;
}

- (NSString*)probeFloatValues:(const std::array<float, kProbeArrayCapacity>&)values
                         count:(int)count {
    NSMutableString* text = [NSMutableString string];
    for (int i = 0; i < count; ++i) {
        if (i) [text appendString:@"|"];
        [text appendFormat:@"%.6g", values[static_cast<std::size_t>(i)]];
    }
    return text;
}

- (NSString*)physicsEdgeValues:(const RuntimePhysicsConfig&)config {
    NSMutableString* text = [NSMutableString string];
    for (int i = 0; i < config.edgeCaptured; ++i) {
        const RuntimePhysicsEdge& edge = config.edges[static_cast<std::size_t>(i)];
        if (!edge.visible) continue;
        if (text.length) [text appendString:@"|"];
        [text appendFormat:@"%d:%.7g:%.7g:%.7g:%.7g",
                           i, edge.start.x, edge.start.y, edge.end.x, edge.end.y];
    }
    return text;
}

- (NSString*)physicsHoleValues:(const RuntimePhysicsConfig&)config {
    NSMutableString* text = [NSMutableString string];
    for (int i = 0; i < config.holeCaptured; ++i) {
        const RuntimePhysicsHole& hole = config.holes[static_cast<std::size_t>(i)];
        if (!hole.visible) continue;
        if (text.length) [text appendString:@"|"];
        [text appendFormat:@"%d:%.7g:%.7g:%.7g:%.7g:%.7g:%.7g:%.7g:%.7g:%.7g:%.7g:%.7g:%.7g:%.7g:%.7g",
                           hole.index, hole.center.x, hole.center.y,
                           hole.leftOffset.x, hole.leftOffset.y,
                           hole.rightOffset.x, hole.rightOffset.y,
                           hole.leftEdge.x, hole.leftEdge.y,
                           hole.rightEdge.x, hole.rightEdge.y,
                           hole.leftDirection.x, hole.leftDirection.y,
                           hole.rightDirection.x, hole.rightDirection.y];
    }
    return text;
}

- (NSString*)nativePhysicsLineValues:(const RuntimePhysicsConfig&)config
                                 hole:(BOOL)hole {
    NSMutableString* text = [NSMutableString string];
    const int count = hole ? config.nativeHoleLineCaptured
                           : config.nativeEdgeLineCaptured;
    const auto& lines = hole ? config.nativeHoleLines : config.nativeEdgeLines;
    for (int i = 0; i < count; ++i) {
        const RuntimePhysicsEdge& line = lines[static_cast<std::size_t>(i)];
        if (!line.visible) continue;
        if (text.length) [text appendString:@"|"];
        [text appendFormat:@"%d:%.7g:%.7g:%.7g:%.7g", i,
                           line.start.x, line.start.y, line.end.x, line.end.y];
    }
    return text;
}

- (NSString*)safeLogToken:(const std::string&)value {
    NSString* text = [NSString stringWithUTF8String:value.c_str()] ?: @"";
    NSMutableString* safe = [text mutableCopy];
    [safe replaceOccurrencesOfString:@"," withString:@"_"
                             options:0 range:NSMakeRange(0, safe.length)];
    [safe replaceOccurrencesOfString:@"|" withString:@"_"
                             options:0 range:NSMakeRange(0, safe.length)];
    [safe replaceOccurrencesOfString:@":" withString:@"_"
                             options:0 range:NSMakeRange(0, safe.length)];
    return safe;
}

- (NSString*)ballMapValues {
    NSMutableString* text = [NSMutableString string];
    const int declaredCount = _snapshot.coordinateBallCount > 0
        ? _snapshot.coordinateBallCount : static_cast<int>(_snapshot.balls.size());
    const int count = std::max(
        0, std::min(static_cast<int>(_snapshot.balls.size()), declaredCount));
    for (int i = 0; i < count; ++i) {
        const RuntimeBall& ball = _snapshot.balls[static_cast<std::size_t>(i)];
        if (text.length) [text appendString:@"|"];
        const float transformDelta = ball.transformVisible
            ? length(Vec2{ball.world.x - ball.transformWorld.x,
                          ball.world.y - ball.transformWorld.y})
            : -1.0f;
        [text appendFormat:@"%d:%@:%d:%.7g:%.7g:%d:%.7g:%.7g:%.7g:%@",
             i, [self safeLogToken:ball.typeName], ball.visible ? 1 : 0,
             ball.world.x, ball.world.y, ball.transformVisible ? 1 : 0,
             ball.transformWorld.x, ball.transformWorld.y, transformDelta,
             [self safeLogToken:ball.name]];
    }
    return text;
}

- (NSString*)trajectoryRouteValues:(const TrajectoryRoute&)route {
    NSMutableString* text = [NSMutableString string];
    const int count = std::max(
        0, std::min(static_cast<int>(kTrajectoryRouteCapacity), route.count));
    for (int i = 0; i < count; ++i) {
        const Segment2& segment = route.segments[static_cast<std::size_t>(i)];
        if (!segment.valid) continue;
        if (text.length) [text appendString:@"|"];
        [text appendFormat:@"%d:%.7g:%.7g:%.7g:%.7g", i,
                           segment.a.x, segment.a.y, segment.b.x, segment.b.y];
    }
    return text;
}

- (NSString*)worldPolylineValues:(const WorldPolyline&)route {
    NSMutableString* text = [NSMutableString string];
    const int count = std::max(
        0, std::min(static_cast<int>(kPostCollisionPointCapacity), route.count));
    for (int i = 0; i < count; ++i) {
        if (i) [text appendString:@"/"];
        const Vec2 point = route.points[static_cast<std::size_t>(i)];
        [text appendFormat:@"%.7g~%.7g", point.x, point.y];
    }
    return text;
}

- (NSString*)nativePhysicsRouteValues:(const RuntimeNativePhysicsProbe&)probe {
    // Cached samples keep availability/status but omit the large payload. Each
    // fresh 10 Hz engine call is still self-contained in one CSV row.
    if (!probe.callAttempted) return @"";
    NSMutableString* text = [NSMutableString string];
    for (std::size_t routeIndex = 0;
         routeIndex < probe.routes.size(); ++routeIndex) {
        const RuntimeNativePhysicsRoute& route = probe.routes[routeIndex];
        if (!route.valid) continue;
        if (text.length) [text appendString:@"|"];
        [text appendFormat:@"r=%zu;b=%d;t=%d;tp=", routeIndex,
                           route.ballIndex, route.trajectoryPointCount];
        for (int i = 0; i < route.trajectoryPointCaptured; ++i) {
            if (i) [text appendString:@"/"];
            const Vec2 point = route.trajectory[static_cast<std::size_t>(i)];
            [text appendFormat:@"%.7g~%.7g", point.x, point.y];
        }
        [text appendFormat:@";c=%d;cp=", route.collisionPointCount];
        for (int i = 0; i < route.collisionPointCaptured; ++i) {
            if (i) [text appendString:@"/"];
            const Vec2 point = route.collisionPoints[static_cast<std::size_t>(i)];
            [text appendFormat:@"%.7g~%.7g", point.x, point.y];
        }
        [text appendFormat:@";h=%d;hb=", route.collisionBallCount];
        for (int i = 0; i < route.collisionBallCaptured; ++i) {
            if (i) [text appendString:@"/"];
            [text appendFormat:@"%d",
                route.collisionBalls[static_cast<std::size_t>(i)]];
        }
    }
    return text;
}

- (NSString*)legacyPhysicsRouteValues:(const RuntimeLegacyCollisionProbe&)probe {
    if (!probe.callAttempted) return @"";
    NSMutableString* text = [NSMutableString string];
    for (int i = 0; i < probe.trajectoryPointCaptured; ++i) {
        if (i) [text appendString:@"/"];
        const Vec2 point = probe.trajectory[static_cast<std::size_t>(i)];
        [text appendFormat:@"%.7g~%.7g", point.x, point.y];
    }
    return text;
}

- (NSString*)nativeEngineBallValues:(const RuntimeNativePhysicsProbe&)probe {
    if (!probe.callAttempted || !probe.stateAttempted) return @"";
    NSMutableString* text = [NSMutableString string];
    for (std::size_t index = 0; index < probe.engineBalls.size(); ++index) {
        const RuntimeNativeEngineBall& ball = probe.engineBalls[index];
        if (!ball.positionValid && !ball.speedValid) continue;
        if (text.length) [text appendString:@"|"];
        [text appendFormat:@"%zu:%.7g:%.7g:%.7g:%.7g", index,
                           ball.position.x, ball.position.y,
                           ball.speed.x, ball.speed.y];
    }
    return text;
}

- (NSString*)nativeRawRouteValues:(const RuntimeNativePhysicsProbe&)probe {
    if (!probe.callAttempted) return @"";
    NSMutableString* text = [NSMutableString string];
    for (std::size_t routeIndex = 0;
         routeIndex < probe.routes.size(); ++routeIndex) {
        const RuntimeNativePhysicsRoute& route = probe.routes[routeIndex];
        if (text.length) [text appendString:@"|"];
        [text appendFormat:@"r=%zu;b=%d;x=%d;y=%d;cx=%d;cy=%d;cb=%d",
                           routeIndex, route.ballIndex,
                           route.rawXTrajectorySize, route.rawYTrajectorySize,
                           route.rawXCollisionSize, route.rawYCollisionSize,
                           route.rawCollisionBallSize];
    }
    return text;
}

- (NSString*)nativeLayoutRouteValues:(const RuntimeNativePhysicsProbe&)probe {
    if (!probe.callAttempted) return @"";
    NSMutableString* text = [NSMutableString string];
    auto appendCandidate = [&](NSString* label,
                               const RuntimeNativeRouteCandidate& candidate) {
        [text appendFormat:@";%@=%d:%d:%d:%d:%d:%d:%d:%d:%d:%d:%d:%d:%d:%d",
            label, candidate.captured ? 1 : 0, candidate.rawBallIndex,
            candidate.selfValid ? 1 : 0, candidate.ballIndex,
            candidate.xTrajectorySize, candidate.yTrajectorySize,
            candidate.xCollisionSize, candidate.yCollisionSize,
            candidate.collisionBallSize, candidate.getterAttempted ? 1 : 0,
            candidate.getterAvailable ? 1 : 0,
            candidate.getterValid ? 1 : 0,
            candidate.getterTrajectoryCount,
            candidate.getterCollisionCount];
    };
    for (std::size_t routeIndex = 0;
         routeIndex < kNativePhysicsRouteCapacity; ++routeIndex) {
        if (text.length) [text appendString:@"|"];
        [text appendFormat:@"r=%zu;hdr=%d;src=%d", routeIndex,
            probe.getDataObjectHeaderMatches[routeIndex] ? 1 : 0,
            probe.selectedRouteSources[routeIndex]];
        appendCandidate(@"result", probe.resultCandidates[routeIndex]);
        appendCandidate(@"array", probe.arrayCandidates[routeIndex]);
        appendCandidate(@"base", probe.getDataBaseCandidates[routeIndex]);
        appendCandidate(@"payload", probe.getDataPayloadCandidates[routeIndex]);
    }
    return text;
}

- (NSString*)nativeSelectedSourceValues:(const RuntimeNativePhysicsProbe&)probe {
    if (!probe.callAttempted) return @"";
    NSMutableString* text = [NSMutableString string];
    for (std::size_t routeIndex = 0;
         routeIndex < kNativePhysicsRouteCapacity; ++routeIndex) {
        if (routeIndex) [text appendString:@"|"];
        [text appendFormat:@"%zu:%d", routeIndex,
            probe.selectedRouteSources[routeIndex]];
    }
    return text;
}

- (void)appendProbeSampleAtTimestamp:(CFTimeInterval)timestamp {
    if (!_recording) return;
    if (_lastProbeSampleTimestamp > 0.0 &&
        timestamp - _lastProbeSampleTimestamp < kProbeSampleInterval) return;
    _lastProbeSampleTimestamp = timestamp;
    if (_recordStartTimestamp <= 0.0) _recordStartTimestamp = timestamp;
    const CFTimeInterval elapsed = timestamp - _recordStartTimestamp;
    if (elapsed >= kProbeMaximumDuration) {
        [self stopRecording];
        _logStatusLabel.text = [NSString stringWithFormat:@"日志：90秒自动停止 %lu 条",
                                                          (unsigned long)_logRowCount];
        return;
    }

    const RuntimeProbe& probe = _snapshot.probe;
    const RuntimeBall& cue = _snapshot.balls[0];
    if (_snapshot.prediction.targetIndex >= 0 &&
        _snapshot.prediction.targetIndex < static_cast<int>(kBallCapacity))
        _recordedTargetIndex = _snapshot.prediction.targetIndex;
    const int targetIndex = _recordedTargetIndex;
    const bool targetValid = targetIndex >= 0 &&
                             targetIndex < static_cast<int>(kBallCapacity) &&
                              _snapshot.balls[static_cast<std::size_t>(targetIndex)].visible;
    const RuntimeBall* target = targetValid
        ? &_snapshot.balls[static_cast<std::size_t>(targetIndex)] : nullptr;
    NSString* linePoints = [self probeLinePoints:probe];
    NSString* xValues = [self probeFloatValues:probe.xPos count:probe.xPosCaptured];
    NSString* yValues = [self probeFloatValues:probe.yPos count:probe.yPosCaptured];
    const RuntimePhysicsConfig& physics = _snapshot.physicsConfig;
    NSString* physicsEdges = [self physicsEdgeValues:physics];
    NSString* physicsHoles = [self physicsHoleValues:physics];
    NSString* nativeEdgeLines = [self nativePhysicsLineValues:physics hole:NO];
    NSString* nativeHoleLines = [self nativePhysicsLineValues:physics hole:YES];
    NSString* ballMap = [self ballMapValues];
    NSString* cueRoute = [self trajectoryRouteValues:_snapshot.prediction.cueApproachRoute];
    NSString* cueAfterRoute = [self trajectoryRouteValues:_snapshot.prediction.cueAfterRoute];
    NSString* targetRoute = [self trajectoryRouteValues:_snapshot.prediction.targetRoute];
    NSString* nativeRoutes = [self nativePhysicsRouteValues:_snapshot.nativePhysicsProbe];
    NSString* legacyPoints = [self legacyPhysicsRouteValues:
        _snapshot.nativePhysicsProbe.legacy];
    NSString* legacyExPoints = [self legacyPhysicsRouteValues:
        _snapshot.nativePhysicsProbe.legacyEx];
    NSString* nativeEngineBalls = [self nativeEngineBallValues:_snapshot.nativePhysicsProbe];
    NSString* nativeRawRoutes = [self nativeRawRouteValues:_snapshot.nativePhysicsProbe];
    NSString* nativeLayoutRoutes = [self nativeLayoutRouteValues:_snapshot.nativePhysicsProbe];
    NSString* nativeSelectedSources =
        [self nativeSelectedSourceValues:_snapshot.nativePhysicsProbe];
    NSString* cuePostRoute = [self worldPolylineValues:
        _snapshot.postCollisionPrediction.cueRoute];
    NSString* objectPostRoute = [self worldPolylineValues:
        _snapshot.postCollisionPrediction.objectRoute];
    NSString* boundsSource = physics.coordinateBoundsReady
        ? @"physics" : (physics.usedPocketFallback ? @"pockets" : @"default");
    Bounds2 railBounds = _snapshot.tableBounds;
    const float activeRailInset = _useOuterRailBoundary ? 0.0f : _snapshot.ballRadius;
    railBounds.min = railBounds.min + Vec2{activeRailInset, activeRailInset};
    railBounds.max = railBounds.max - Vec2{activeRailInset, activeRailInset};
    NSString* baseLine = [NSString stringWithFormat:
        @"%.3f,%d,%d,%d,%d,%.7g,%.7g,%.7g,%.7g,%.7g,%.7g,%.7g,%.7g,%.7g,%.7g,"
         @"%d,%d,%.7g,%d,%@,%d,%@,%d,%@,%d,%.7g,%.7g,%d,%d,%.7g,%.7g,"
         @"%d,%d,%d,%@,%d,%@,%.7g,%.7g,%.7g,%.7g,%.7g,%.7g,"
         @"%@,%.7g,%.7g,%.7g,%.7g,%.7g,%.7g,%.7g,%.7g",
        elapsed * 1000.0, _snapshot.runtimeReady ? 1 : 0, probe.available ? 1 : 0,
        probe.isShowLine ? 1 : 0, _snapshot.aimingActive ? 1 : 0,
        probe.forceValue, probe.forceDuration, probe.shotForce, probe.addedForce,
        probe.shootValue, probe.shootForce, probe.xSpin, probe.ySpin,
        probe.xMouse, probe.yMouse, probe.degree, probe.randIc, probe.trace,
        probe.lineDataCount, linePoints, probe.xPosCount, xValues,
        probe.yPosCount, yValues, cue.visible ? 1 : 0, cue.world.x, cue.world.y,
        targetIndex, targetValid ? 1 : 0,
        target ? target->world.x : 0.0f, target ? target->world.y : 0.0f,
        physics.modelFound ? 1 : 0, physics.available ? 1 : 0,
        physics.edgeCount, physicsEdges, physics.holeCount, physicsHoles,
        physics.coordinateWidth, physics.coordinateHeight,
        physics.coordinateScale, physics.coordinateOffset.x, physics.coordinateOffset.y,
        physics.ballScreenRadius, boundsSource,
        _snapshot.tableBounds.min.x, _snapshot.tableBounds.min.y,
        _snapshot.tableBounds.max.x, _snapshot.tableBounds.max.y,
        railBounds.min.x, railBounds.min.y, railBounds.max.x, railBounds.max.y];

    const CFTimeInterval shotElapsed = _shotStartTimestamp > 0.0
        ? timestamp - _shotStartTimestamp : -1.0;
    NSString* shotPhase = _snapshot.aimingActive
        ? @"aiming"
        : ((shotElapsed >= 0.0 && shotElapsed <= 5.0) ? @"post_shot" : @"idle");
    auto integerText = [](long long value) {
        return [NSString stringWithFormat:@"%lld", value];
    };
    auto floatText = [](double value) {
        return [NSString stringWithFormat:@"%.7g", value];
    };
    NSMutableArray<NSString*>* extra = [NSMutableArray arrayWithCapacity:64];
    [extra addObject:[self safeLogToken:_snapshot.gameMode]];
    [extra addObject:[self safeLogToken:_snapshot.ballPositionSource]];
    [extra addObject:integerText(_snapshot.coordinateBallCount)];
    [extra addObject:integerText(_snapshot.activeBallCount)];
    [extra addObject:ballMap];
    [extra addObject:[self safeLogToken:_snapshot.aimSource]];
    [extra addObject:floatText(_snapshot.aimDirection.x)];
    [extra addObject:floatText(_snapshot.aimDirection.y)];
    [extra addObject:integerText(_snapshot.lastAimAvailable ? 1 : 0)];
    [extra addObject:floatText(_snapshot.lastAimDirection.x)];
    [extra addObject:floatText(_snapshot.lastAimDirection.y)];
    [extra addObject:integerText(_snapshot.crosshairAimAvailable ? 1 : 0)];
    [extra addObject:floatText(_snapshot.crosshairAimDirection.x)];
    [extra addObject:floatText(_snapshot.crosshairAimDirection.y)];
    [extra addObject:floatText(_snapshot.crosshairWorld.x)];
    [extra addObject:floatText(_snapshot.crosshairWorld.y)];
    [extra addObject:floatText(_snapshot.crosshairLastAngleDeltaDegrees)];
    [extra addObject:integerText(_snapshot.gameLineAvailable ? 1 : 0)];
    [extra addObject:floatText(_snapshot.gameLineScreenDirection.x)];
    [extra addObject:floatText(_snapshot.gameLineScreenDirection.y)];
    [extra addObject:floatText(_snapshot.gameLineAimDeltaDegrees)];
    [extra addObject:integerText(_shotId)];
    [extra addObject:shotPhase];
    [extra addObject:[NSString stringWithFormat:@"%.3f", shotElapsed * 1000.0]];
    [extra addObject:floatText(_bounceAngleOffsetDegrees)];
    [extra addObject:floatText(_secondaryBounceAngleOffsetDegrees)];
    [extra addObject:integerText(_secondaryBounceAngleLinked ? 1 : 0)];
    [extra addObject:integerText(_maximumRailBounces)];
    [extra addObject:_useOuterRailBoundary ? @"dashed_outer" : @"ball_center"];
    [extra addObject:cueRoute];
    [extra addObject:cueAfterRoute];
    [extra addObject:targetRoute];
    const RuntimeNativePhysicsProbe& native = _snapshot.nativePhysicsProbe;
    [extra addObject:integerText(native.classFound ? 1 : 0)];
    [extra addObject:integerText(native.methodFound ? 1 : 0)];
    [extra addObject:integerText(native.callEligible ? 1 : 0)];
    [extra addObject:integerText(native.stateMethodsFound ? 1 : 0)];
    [extra addObject:integerText(native.stateAttempted ? 1 : 0)];
    [extra addObject:integerText(native.stateAvailable ? 1 : 0)];
    [extra addObject:integerText(native.engineBallCount)];
    [extra addObject:integerText(native.engineBallType)];
    [extra addObject:integerText(native.enginePositionCount)];
    [extra addObject:integerText(native.transformComparisonCount)];
    [extra addObject:floatText(native.transformMaximumDelta)];
    [extra addObject:nativeEngineBalls];
    [extra addObject:integerText(native.modeFieldsFound ? 1 : 0)];
    [extra addObject:integerText(native.modeValuesAvailable ? 1 : 0)];
    [extra addObject:integerText(native.useEx ? 1 : 0)];
    [extra addObject:integerText(native.useExV0 ? 1 : 0)];
    [extra addObject:integerText(native.useExV1 ? 1 : 0)];
    [extra addObject:integerText(native.bothMode ? 1 : 0)];
    const RuntimeLegacyCollisionProbe& legacy = native.legacy;
    [extra addObject:integerText(native.legacyMethodFound ? 1 : 0)];
    [extra addObject:integerText(legacy.callAttempted ? 1 : 0)];
    [extra addObject:integerText(legacy.directSretChangedBytes)];
    [extra addObject:integerText(legacy.requestedCollisionCount)];
    [extra addObject:integerText(legacy.reportedPointCount)];
    [extra addObject:integerText(legacy.scannedPointCount)];
    [extra addObject:integerText(legacy.trajectoryPointCaptured)];
    [extra addObject:integerText(legacy.collisionBall)];
    [extra addObject:floatText(legacy.maximumChordResidual)];
    [extra addObject:integerText(legacy.valid ? 1 : 0)];
    [extra addObject:[self safeLogToken:legacy.status]];
    [extra addObject:legacyPoints];
    const RuntimeLegacyCollisionProbe& legacyEx = native.legacyEx;
    [extra addObject:integerText(native.legacyExMethodFound ? 1 : 0)];
    [extra addObject:integerText(legacyEx.callAttempted ? 1 : 0)];
    [extra addObject:integerText(legacyEx.directSretChangedBytes)];
    [extra addObject:integerText(legacyEx.requestedCollisionCount)];
    [extra addObject:integerText(legacyEx.reportedPointCount)];
    [extra addObject:integerText(legacyEx.scannedPointCount)];
    [extra addObject:integerText(legacyEx.trajectoryPointCaptured)];
    [extra addObject:integerText(legacyEx.collisionBall)];
    [extra addObject:floatText(legacyEx.maximumChordResidual)];
    [extra addObject:integerText(legacyEx.valid ? 1 : 0)];
    [extra addObject:[self safeLogToken:legacyEx.status]];
    [extra addObject:legacyExPoints];
    [extra addObject:integerText(native.managedParserFound ? 1 : 0)];
    [extra addObject:integerText(native.managedValidCountAvailable ? 1 : 0)];
    [extra addObject:integerText(native.managedValidCount)];
    [extra addObject:integerText(native.valueBoxAvailable ? 1 : 0)];
    [extra addObject:integerText(native.managedArrayMethodFound ? 1 : 0)];
    [extra addObject:integerText(native.managedArrayAvailable ? 1 : 0)];
    [extra addObject:integerText(native.managedArrayLength)];
    [extra addObject:integerText(native.directSretUsed ? 1 : 0)];
    [extra addObject:integerText(native.directSretChangedBytes)];
    [extra addObject:integerText(native.directCollisionValidCount)];
    [extra addObject:integerText(native.directValidCount)];
    [extra addObject:nativeRawRoutes];
    [extra addObject:nativeLayoutRoutes];
    [extra addObject:nativeSelectedSources];
    [extra addObject:integerText(native.callAttempted ? 1 : 0)];
    [extra addObject:integerText(native.available ? 1 : 0)];
    [extra addObject:[self safeLogToken:native.status]];
    [extra addObject:floatText(native.inputForce)];
    [extra addObject:floatText(native.inputXSpin)];
    [extra addObject:floatText(native.inputYSpin)];
    [extra addObject:floatText(-native.inputXSpin)];
    [extra addObject:floatText(native.inputYSpin)];
    [extra addObject:floatText(native.inputXMouse)];
    [extra addObject:floatText(native.inputYMouse)];
    [extra addObject:integerText(native.inputDegree)];
    [extra addObject:integerText(native.validRouteCount)];
    [extra addObject:nativeRoutes];
    [extra addObject:integerText(physics.nativeLineMethodsFound ? 1 : 0)];
    [extra addObject:integerText(physics.nativeLinesAvailable ? 1 : 0)];
    [extra addObject:integerText(physics.nativeEdgeLineCount)];
    [extra addObject:nativeEdgeLines];
    [extra addObject:integerText(physics.nativeHoleLineCount)];
    [extra addObject:nativeHoleLines];
    const double exactForcePercent = std::max(0.0, std::min(
        100.0, static_cast<double>(_liveForce) /
        static_cast<double>(kObservedMaximumShootForce) * 100.0));
    const double forceSampleAgeMs = _lastForceSampleTimestamp > 0.0
        ? (timestamp - _lastForceSampleTimestamp) * 1000.0 : -1.0;
    [extra addObject:floatText(_liveForce)];
    [extra addObject:floatText(_hasReleaseForce ? _releaseForce : 0.0f)];
    [extra addObject:floatText(exactForcePercent)];
    [extra addObject:floatText(forceSampleAgeMs)];
    [extra addObject:integerText(native.firstCollisionSpeedMethodFound ? 1 : 0)];
    [extra addObject:integerText(native.firstCollisionSpeedAttempted ? 1 : 0)];
    [extra addObject:integerText(native.firstCollisionSpeedAvailable ? 1 : 0)];
    [extra addObject:floatText(native.firstCollisionSpeedRaw.x)];
    [extra addObject:floatText(native.firstCollisionSpeedRaw.y)];
    [extra addObject:floatText(native.firstCollisionSpeedWorld.x)];
    [extra addObject:floatText(native.firstCollisionSpeedWorld.y)];
    [extra addObject:integerText(native.firstCollisionSpeedBallIndex)];
    [extra addObject:integerText(native.targetCollisionSpeedAttempted ? 1 : 0)];
    [extra addObject:integerText(native.targetCollisionSpeedAvailable ? 1 : 0)];
    [extra addObject:integerText(native.targetCollisionSpeedBallIndex)];
    [extra addObject:floatText(native.targetCollisionSpeedRaw.x)];
    [extra addObject:floatText(native.targetCollisionSpeedRaw.y)];
    [extra addObject:floatText(native.targetCollisionSpeedWorld.x)];
    [extra addObject:floatText(native.targetCollisionSpeedWorld.y)];
    const PostCollisionPrediction& post = _snapshot.postCollisionPrediction;
    [extra addObject:integerText(post.valid ? 1 : 0)];
    [extra addObject:integerText(post.firstTargetIndex)];
    [extra addObject:floatText(post.impactPoint.x)];
    [extra addObject:floatText(post.impactPoint.y)];
    [extra addObject:floatText(post.incomingSpeed)];
    [extra addObject:post.incomingSpeedFromNative ? @"native" : @"force_formula"];
    [extra addObject:floatText(post.fallbackIncomingSpeed)];
    [extra addObject:floatText(post.probedIncomingSpeed)];
    [extra addObject:integerText(post.nativeCueAfterVelocityUsed ? 1 : 0)];
    [extra addObject:integerText(post.nativeObjectAfterVelocityUsed ? 1 : 0)];
    [extra addObject:integerText(post.objectCollisionCount)];
    [extra addObject:integerText(post.objectRailCount)];
    [extra addObject:floatText(worldPolylineLength(post.cueRoute))];
    [extra addObject:floatText(worldPolylineLength(post.objectRoute))];
    const Vec2 cueStop = post.cueRoute.count > 0
        ? post.cueRoute.points[static_cast<std::size_t>(post.cueRoute.count - 1)]
        : Vec2{};
    const Vec2 objectStop = post.objectRoute.count > 0
        ? post.objectRoute.points[static_cast<std::size_t>(post.objectRoute.count - 1)]
        : Vec2{};
    [extra addObject:floatText(cueStop.x)];
    [extra addObject:floatText(cueStop.y)];
    [extra addObject:floatText(objectStop.x)];
    [extra addObject:floatText(objectStop.y)];
    [extra addObject:cuePostRoute];
    [extra addObject:objectPostRoute];
    NSString* line = [NSString stringWithFormat:@"%@,%@\n", baseLine,
                      [extra componentsJoinedByString:@","]];
    [_pendingLogLines addObject:line];
    ++_logRowCount;
    if (_pendingLogLines.count >= 60) [self flushPendingLogLines];
    if (_menuVisible && (_logRowCount % 15 == 0)) {
        _logStatusLabel.text = [NSString stringWithFormat:@"日志：记录中 %lu 条 / %.1f秒",
                                                          (unsigned long)_logRowCount, elapsed];
    }
}

- (void)exportLog {
    if (_recording) [self stopRecording];
    NSString* csvPath = [_currentCSVPath copy];
    NSString* metadataPath = [_currentMetadataPath copy];
    if (csvPath.length == 0 || ![NSFileManager.defaultManager fileExistsAtPath:csvPath]) {
        NSString* directory = [self probeLogDirectory];
        NSArray<NSString*>* names = [[NSFileManager.defaultManager
            contentsOfDirectoryAtPath:directory error:nil]
            sortedArrayUsingSelector:@selector(compare:)];
        for (NSString* name in names.reverseObjectEnumerator) {
            if (![name.pathExtension.lowercaseString isEqualToString:@"csv"]) continue;
            csvPath = [directory stringByAppendingPathComponent:name];
            metadataPath = [[csvPath stringByDeletingPathExtension] stringByAppendingPathExtension:@"txt"];
            break;
        }
    }
    if (csvPath.length == 0) {
        _logStatusLabel.text = @"日志：没有可导出的文件";
        return;
    }

    __weak PoolOverlayView* weakSelf = self;
    dispatch_async(_logQueue, ^{
        dispatch_async(dispatch_get_main_queue(), ^{
            PoolOverlayView* strongSelf = weakSelf;
            if (!strongSelf || strongSelf->_shuttingDown) return;
            NSMutableArray<NSURL*>* items = [NSMutableArray arrayWithObject:[NSURL fileURLWithPath:csvPath]];
            if ([NSFileManager.defaultManager fileExistsAtPath:metadataPath])
                [items addObject:[NSURL fileURLWithPath:metadataPath]];
            UIDocumentPickerViewController* picker = nil;
            if (@available(iOS 14.0, *)) {
                picker = [[UIDocumentPickerViewController alloc] initForExportingURLs:items
                                                                               asCopy:YES];
            } else {
                picker = [[UIDocumentPickerViewController alloc] initWithURLs:items
                                                                        inMode:UIDocumentPickerModeExportToService];
            }
            picker.allowsMultipleSelection = YES;
            picker.popoverPresentationController.sourceView = strongSelf->_exportButton;
            picker.popoverPresentationController.sourceRect = strongSelf->_exportButton.bounds;
            UIViewController* presenter = strongSelf.window.rootViewController;
            while (presenter.presentedViewController) presenter = presenter.presentedViewController;
            [presenter presentViewController:picker animated:YES completion:nil];
            strongSelf->_logStatusLabel.text = @"日志：已打开文件导出";
        });
    });
}

- (void)clearLogs {
    if (_recording) [self stopRecording];
    _currentCSVPath = nil;
    _currentMetadataPath = nil;
    NSString* directory = [[self probeLogDirectory] copy];
    __weak PoolOverlayView* weakSelf = self;
    dispatch_async(_logQueue, ^{
        NSArray<NSString*>* names = [NSFileManager.defaultManager
            contentsOfDirectoryAtPath:directory error:nil];
        for (NSString* name in names) {
            NSString* path = [directory stringByAppendingPathComponent:name];
            [NSFileManager.defaultManager removeItemAtPath:path error:nil];
        }
        dispatch_async(dispatch_get_main_queue(), ^{
            PoolOverlayView* strongSelf = weakSelf;
            if (strongSelf && !strongSelf->_shuttingDown)
                strongSelf->_logStatusLabel.text = @"日志：已清空";
        });
    });
}

- (UIView*)hitTest:(CGPoint)point withEvent:(UIEvent*)event {
    UIView* hit = [super hitTest:point withEvent:event];
    return hit == self ? nil : hit;
}

- (void)updateFrame:(CADisplayLink*)sender {
    if (_shuttingDown || _samplingSuspended || !_bridge) return;
    if (UIApplication.sharedApplication.applicationState != UIApplicationStateActive) return;
    if (_fpsWindowStart <= 0.0) _fpsWindowStart = sender.timestamp;
    ++_fpsFrameCount;
    const CFTimeInterval elapsed = sender.timestamp - _fpsWindowStart;
    if (elapsed >= 0.75) {
        _measuredFps = static_cast<NSInteger>(std::lround(_fpsFrameCount / elapsed));
        _fpsFrameCount = 0;
        _fpsWindowStart = sender.timestamp;
    }
    _snapshot = _bridge->sample();
    if (_hasPreviousAimState && !_previousAimingActive &&
        _snapshot.aimingActive) {
        _lastNonZeroForce = 0.0f;
    }
    const RuntimeProbe& forceProbe = _snapshot.probe;
    const bool forceValid = forceProbe.shootInfoAvailable &&
        std::isfinite(forceProbe.shootForce) && forceProbe.shootForce >= 0.0f &&
        forceProbe.shootForce <= 20.0f;
    const float currentForce = forceValid ? forceProbe.shootForce : 0.0f;
    if (currentForce > 0.001f) {
        _liveForce = currentForce;
        _lastNonZeroForce = currentForce;
        _lastForceSampleTimestamp = sender.timestamp;
    } else {
        _liveForce = 0.0f;
    }
    if (_hasPreviousAimState && _previousAimingActive && !_snapshot.aimingActive) {
        if (_lastNonZeroForce > 0.001f) {
            _releaseForce = _lastNonZeroForce;
            _hasReleaseForce = YES;
        }
        ++_shotId;
        _shotStartTimestamp = sender.timestamp;
    }
    _previousAimingActive = _snapshot.aimingActive;
    _hasPreviousAimState = YES;
    if (forceProbe.available) {
        NSString* releaseText = _hasReleaseForce
            ? [NSString stringWithFormat:@"%.6f", _releaseForce] : @"--";
        _forceHudLabel.text = [NSString stringWithFormat:
            @"实时 %.6f  |  出杆 %@", _liveForce, releaseText];
        _forceHudLabel.textColor = _liveForce > 0.001f
            ? [UIColor colorWithRed:1.0 green:0.72 blue:0.16 alpha:1.0]
            : UIColor.whiteColor;
    } else {
        _forceHudLabel.text = @"力度：等待球桌数据";
        _forceHudLabel.textColor = UIColor.whiteColor;
    }
    [self appendProbeSampleAtTimestamp:sender.timestamp];
    if (_menuVisible) {
        NSUInteger ballCount = 0;
        for (const RuntimeBall& ball : _snapshot.balls) if (ball.visible) ++ballCount;
        _statusLabel.text = [NSString stringWithFormat:
            @"EX 橙线点 %d / 球 %lu / 刷新 %ldHz",
            _snapshot.nativePhysicsProbe.legacyEx.trajectoryPointCaptured,
            (unsigned long)ballCount, (long)_measuredFps];
        _versionLabel.text = kPoolVisibleVersion;
    }
    [self setNeedsDisplay];
}

- (CGPoint)convertUnityPoint:(Vec3)point {
    const CGFloat width = CGRectGetWidth(self.bounds);
    const CGFloat height = CGRectGetHeight(self.bounds);
    if (_snapshot.unityScreenWidth <= 0 || _snapshot.unityScreenHeight <= 0) return CGPointZero;
    const CGFloat x = point.x / static_cast<CGFloat>(_snapshot.unityScreenWidth) * width;
    const CGFloat y = height - point.y / static_cast<CGFloat>(_snapshot.unityScreenHeight) * height;
    return CGPointMake(x, y);
}

- (void)strokeSegment:(RuntimeScreenSegment)segment
               color:(UIColor*)color
               width:(CGFloat)width
             context:(CGContextRef)context {
    if (!segment.visible) return;
    const CGPoint a = [self convertUnityPoint:segment.a];
    const CGPoint b = [self convertUnityPoint:segment.b];
    CGContextSetStrokeColorWithColor(context, color.CGColor);
    CGContextSetLineWidth(context, width);
    CGContextSetLineCap(context, kCGLineCapRound);
    CGContextMoveToPoint(context, a.x, a.y);
    CGContextAddLineToPoint(context, b.x, b.y);
    CGContextStrokePath(context);
}

- (void)strokeRoute:(const RuntimeScreenRoute&)route
               color:(UIColor*)color
               width:(CGFloat)width
             context:(CGContextRef)context {
    const int count = std::max(
        0, std::min(static_cast<int>(kTrajectoryRouteCapacity), route.count));
    for (int i = 0; i < count; ++i) {
        [self strokeSegment:route.segments[static_cast<std::size_t>(i)]
                      color:color width:width context:context];
    }
}

- (void)strokePolyline:(const RuntimeScreenPolyline&)route
                  color:(UIColor*)color
                  width:(CGFloat)width
                 dashed:(BOOL)dashed
                context:(CGContextRef)context {
    const int count = std::max(0, std::min(
        static_cast<int>(kLegacyNativeTrajectoryCapacity), route.count));
    if (!route.visible || count < 2) return;
    CGContextSetStrokeColorWithColor(context, color.CGColor);
    CGContextSetLineWidth(context, width);
    CGContextSetLineCap(context, kCGLineCapRound);
    CGContextSetLineJoin(context, kCGLineJoinRound);
    if (dashed) {
        const CGFloat pattern[] = {7.0, 4.0};
        CGContextSetLineDash(context, 0.0, pattern, 2);
    } else {
        CGContextSetLineDash(context, 0.0, nullptr, 0);
    }
    const CGPoint first = [self convertUnityPoint:route.points[0]];
    CGContextMoveToPoint(context, first.x, first.y);
    for (int i = 1; i < count; ++i) {
        const CGPoint point = [self convertUnityPoint:
            route.points[static_cast<std::size_t>(i)]];
        CGContextAddLineToPoint(context, point.x, point.y);
    }
    CGContextStrokePath(context);
    CGContextSetLineDash(context, 0.0, nullptr, 0);
}

- (void)drawRect:(CGRect)rect {
    (void)rect;
    CGContextRef context = UIGraphicsGetCurrentContext();
    if (!context) return;

    CGRect calibratedTableRect = CGRectNull;
    {
        const CGRect bounds = self.bounds;
        std::array<CGPoint, 6> pocketPoints{};
        bool haveAllPockets = true;
        for (std::size_t i = 0; i < pocketPoints.size(); ++i) {
            const RuntimePocket& pocket = _snapshot.pockets[i];
            if (!pocket.visible || pocket.screenPixels.z <= 0.0f) {
                haveAllPockets = false;
                break;
            }
            pocketPoints[i] = [self convertUnityPoint:pocket.screenPixels];
        }

        if (!haveAllPockets) {
            const CGRect fallback = CGRectMake(CGRectGetWidth(bounds) * 0.183,
                                               CGRectGetHeight(bounds) * 0.214,
                                               CGRectGetWidth(bounds) * 0.633,
                                               CGRectGetHeight(bounds) * 0.706);
            pocketPoints[0] = CGPointMake(CGRectGetMinX(fallback), CGRectGetMinY(fallback));
            pocketPoints[1] = CGPointMake(CGRectGetMinX(fallback), CGRectGetMaxY(fallback));
            pocketPoints[2] = CGPointMake(CGRectGetMaxX(fallback), CGRectGetMinY(fallback));
            pocketPoints[3] = CGPointMake(CGRectGetMaxX(fallback), CGRectGetMaxY(fallback));
            pocketPoints[4] = CGPointMake(CGRectGetMidX(fallback), CGRectGetMinY(fallback));
            pocketPoints[5] = CGPointMake(CGRectGetMidX(fallback), CGRectGetMaxY(fallback));
        }

        CGFloat minX = pocketPoints[0].x, maxX = pocketPoints[0].x;
        CGFloat minY = pocketPoints[0].y, maxY = pocketPoints[0].y;
        for (CGPoint point : pocketPoints) {
            minX = MIN(minX, point.x); maxX = MAX(maxX, point.x);
            minY = MIN(minY, point.y); maxY = MAX(maxY, point.y);
        }
        const CGPoint center = CGPointMake((minX + maxX) * 0.5, (minY + maxY) * 0.5);
        for (CGPoint& point : pocketPoints) {
            point.x = center.x + (point.x - center.x) * _tableScaleX;
            point.y = center.y + (point.y - center.y) * _tableScaleY;
        }

        CGFloat calibratedMinX = pocketPoints[0].x, calibratedMaxX = pocketPoints[0].x;
        CGFloat calibratedMinY = pocketPoints[0].y, calibratedMaxY = pocketPoints[0].y;
        for (CGPoint point : pocketPoints) {
            calibratedMinX = MIN(calibratedMinX, point.x);
            calibratedMaxX = MAX(calibratedMaxX, point.x);
            calibratedMinY = MIN(calibratedMinY, point.y);
            calibratedMaxY = MAX(calibratedMaxY, point.y);
        }
        calibratedTableRect = CGRectMake(calibratedMinX, calibratedMinY,
                                         calibratedMaxX - calibratedMinX,
                                         calibratedMaxY - calibratedMinY);

        if (_calibrationEnabled) {
        UIColor* frameColor = [UIColor colorWithRed:0.05 green:0.9 blue:1.0 alpha:0.95];
        CGContextSetLineDash(context, 0.0, nullptr, 0);
        CGContextSetLineWidth(context, 2.0);
        CGContextSetStrokeColorWithColor(context, frameColor.CGColor);
        CGContextMoveToPoint(context, pocketPoints[0].x, pocketPoints[0].y);
        CGContextAddLineToPoint(context, pocketPoints[2].x, pocketPoints[2].y);
        CGContextAddLineToPoint(context, pocketPoints[3].x, pocketPoints[3].y);
        CGContextAddLineToPoint(context, pocketPoints[1].x, pocketPoints[1].y);
        CGContextClosePath(context);
        CGContextStrokePath(context);

        const CGFloat basePocketRadius = MAX(8.0, MIN(14.0, CGRectGetWidth(calibratedTableRect) * 0.012));
        const CGFloat pocketRadius = MAX(4.0, MIN(28.0, basePocketRadius * _pocketScale));
        for (CGPoint point : pocketPoints) {
            const CGRect hole = CGRectMake(point.x - pocketRadius, point.y - pocketRadius,
                                           pocketRadius * 2.0, pocketRadius * 2.0);
            CGContextSetFillColorWithColor(context, [UIColor colorWithWhite:0.0 alpha:0.68].CGColor);
            CGContextFillEllipseInRect(context, hole);
            CGContextSetStrokeColorWithColor(context, frameColor.CGColor);
            CGContextSetLineWidth(context, 2.0);
            CGContextStrokeEllipseInRect(context, hole);
        }
        }
    }

    if (_calibrationEnabled && _snapshot.physicsConfig.boundsProjected) {
        const RuntimePhysicsConfig& physics = _snapshot.physicsConfig;
        auto strokeBounds = [&](const std::array<Vec3, 4>& corners,
                                UIColor* color, CGFloat width, bool dashed) {
            for (const Vec3& point : corners) if (point.z <= 0.0f) return;
            CGContextSetStrokeColorWithColor(context, color.CGColor);
            CGContextSetLineWidth(context, width);
            if (dashed) {
                const CGFloat pattern[] = {6.0, 4.0};
                CGContextSetLineDash(context, 0.0, pattern, 2);
            } else {
                CGContextSetLineDash(context, 0.0, nullptr, 0);
            }
            const CGPoint first = [self convertUnityPoint:corners[0]];
            CGContextMoveToPoint(context, first.x, first.y);
            for (std::size_t i = 1; i < corners.size(); ++i) {
                const CGPoint point = [self convertUnityPoint:corners[i]];
                CGContextAddLineToPoint(context, point.x, point.y);
            }
            CGContextClosePath(context);
            CGContextStrokePath(context);
        };
        CGContextSaveGState(context);
        strokeBounds(physics.outerBoundsScreen,
                     [UIColor colorWithRed:1.0 green:0.78 blue:0.05 alpha:0.85], 1.5, true);
        strokeBounds(physics.railBoundsScreen,
                     [UIColor colorWithRed:0.2 green:1.0 blue:0.35 alpha:0.95], 2.0, false);
        CGContextRestoreGState(context);
    }

    if (_calibrationEnabled && _snapshot.physicsConfig.available) {
        CGContextSaveGState(context);
        CGContextSetLineDash(context, 0.0, nullptr, 0);
        const RuntimePhysicsConfig& physics = _snapshot.physicsConfig;
        for (int i = 0; i < physics.edgeCaptured; ++i) {
            const RuntimePhysicsEdge& edge = physics.edges[static_cast<std::size_t>(i)];
            if (!edge.visible || edge.startScreen.z <= 0.0f || edge.endScreen.z <= 0.0f) continue;
            RuntimeScreenSegment segment{edge.startScreen, edge.endScreen, true};
            [self strokeSegment:segment
                          color:[UIColor colorWithRed:1.0 green:0.15 blue:0.85 alpha:0.95]
                          width:2.0 context:context];
        }
        for (int i = 0; i < physics.holeCaptured; ++i) {
            const RuntimePhysicsHole& hole = physics.holes[static_cast<std::size_t>(i)];
            if (!hole.visible || hole.leftEdgeScreen.z <= 0.0f ||
                hole.rightEdgeScreen.z <= 0.0f) continue;
            RuntimeScreenSegment jaw{hole.leftEdgeScreen, hole.rightEdgeScreen, true};
            [self strokeSegment:jaw
                          color:[UIColor colorWithRed:1.0 green:0.55 blue:0.05 alpha:0.95]
                          width:1.5 context:context];
            if (hole.centerScreen.z > 0.0f) {
                const CGPoint center = [self convertUnityPoint:hole.centerScreen];
                CGContextSetStrokeColorWithColor(context,
                    [UIColor colorWithRed:1.0 green:0.55 blue:0.05 alpha:0.95].CGColor);
                CGContextSetLineWidth(context, 1.5);
                CGContextStrokeEllipseInRect(context,
                    CGRectMake(center.x - 4.0, center.y - 4.0, 8.0, 8.0));
            }
        }
        for (int i = 0; i < physics.nativeEdgeLineCaptured; ++i) {
            const RuntimePhysicsEdge& edge =
                physics.nativeEdgeLines[static_cast<std::size_t>(i)];
            if (!edge.visible || edge.startScreen.z <= 0.0f ||
                edge.endScreen.z <= 0.0f) continue;
            RuntimeScreenSegment segment{edge.startScreen, edge.endScreen, true};
            [self strokeSegment:segment
                          color:[UIColor colorWithRed:0.15 green:0.9 blue:1.0 alpha:0.95]
                          width:1.5 context:context];
        }
        for (int i = 0; i < physics.nativeHoleLineCaptured; ++i) {
            const RuntimePhysicsEdge& edge =
                physics.nativeHoleLines[static_cast<std::size_t>(i)];
            if (!edge.visible || edge.startScreen.z <= 0.0f ||
                edge.endScreen.z <= 0.0f) continue;
            RuntimeScreenSegment segment{edge.startScreen, edge.endScreen, true};
            [self strokeSegment:segment
                          color:[UIColor colorWithRed:1.0 green:0.45 blue:0.05 alpha:0.98]
                          width:2.0 context:context];
        }
        CGContextRestoreGState(context);
    }

    // Short/full Snooker can expose more than the old 16-ball capacity. Draw
    // every discovered Snooker ball automatically; the legacy manual marker
    // switch remains disabled in the compact menu.
    if (_ballMarkersEnabled || _snapshot.snookerBallSet) {
        for (const RuntimeBall& ball : _snapshot.balls) {
            if (!ball.visible || ball.screenPixels.z <= 0.0f) continue;
            const CGPoint center = [self convertUnityPoint:ball.screenPixels];
            UIColor* color = ball.index == 0
                ? [UIColor colorWithRed:0.1 green:0.85 blue:1.0 alpha:1.0]
                : [UIColor colorWithRed:0.2 green:1.0 blue:0.45 alpha:0.95];
            if (_predictionEnabled && _snapshot.aimingActive &&
                ball.index == _snapshot.prediction.targetIndex)
                color = [UIColor colorWithRed:1.0 green:0.2 blue:0.75 alpha:1.0];
            CGContextSetStrokeColorWithColor(context, color.CGColor);
            CGContextSetLineWidth(context, 2.0);
            CGContextStrokeEllipseInRect(context,
                CGRectMake(center.x - 7.0, center.y - 7.0, 14.0, 14.0));
            NSString* label = [NSString stringWithFormat:@"%d", ball.index];
            NSDictionary* attrs = @{
                NSFontAttributeName: [UIFont monospacedDigitSystemFontOfSize:10.0
                                                                       weight:UIFontWeightBold],
                NSForegroundColorAttributeName: color
            };
            [label drawAtPoint:CGPointMake(center.x + 8.0, center.y - 7.0)
                withAttributes:attrs];
        }
    }

    if (_predictionEnabled && _snapshot.aimingActive) {
        CGContextSaveGState(context);
        if (!CGRectIsNull(calibratedTableRect) && !CGRectIsEmpty(calibratedTableRect))
            CGContextClipToRect(context, CGRectInset(calibratedTableRect, -2.0, -2.0));
        // The game-native white line remains authoritative through first
        // contact. Draw only the 0.2.3 post-contact extensions here so the
        // table is not covered by a duplicate orange approach line.
        [self strokePolyline:_snapshot.objectPostCollisionScreenRoute
                       color:[UIColor colorWithRed:1.0 green:0.88 blue:0.12 alpha:0.98]
                       width:2.0 dashed:NO context:context];
        [self strokePolyline:_snapshot.cuePostCollisionScreenRoute
                       color:[UIColor colorWithRed:0.12 green:0.92 blue:1.0 alpha:0.98]
                       width:2.0 dashed:YES context:context];

        auto drawLanding = [&](const RuntimeScreenPolyline& route,
                               UIColor* color, NSString* label) {
            if (!route.visible || route.count < 1) return;
            const CGPoint point = [self convertUnityPoint:
                route.points[static_cast<std::size_t>(route.count - 1)]];
            CGContextSetFillColorWithColor(context,
                [UIColor colorWithWhite:0.02 alpha:0.76].CGColor);
            CGContextFillEllipseInRect(context,
                CGRectMake(point.x - 7.0, point.y - 7.0, 14.0, 14.0));
            CGContextSetStrokeColorWithColor(context, color.CGColor);
            CGContextSetLineWidth(context, 2.0);
            CGContextStrokeEllipseInRect(context,
                CGRectMake(point.x - 7.0, point.y - 7.0, 14.0, 14.0));
            [label drawAtPoint:CGPointMake(point.x + 8.0, point.y - 8.0)
                withAttributes:@{
                    NSFontAttributeName: [UIFont monospacedDigitSystemFontOfSize:9.0
                        weight:UIFontWeightBold],
                    NSForegroundColorAttributeName: color
                }];
        };
        drawLanding(_snapshot.objectPostCollisionScreenRoute,
                    [UIColor colorWithRed:1.0 green:0.88 blue:0.12 alpha:0.98],
                    @"OBJ");
        drawLanding(_snapshot.cuePostCollisionScreenRoute,
                    [UIColor colorWithRed:0.12 green:0.92 blue:1.0 alpha:0.98],
                    @"CUE");
        CGContextRestoreGState(context);
    }

}

- (void)dealloc {
    [self shutdown];
}

@end

static UIWindowScene* PoolLabFindScene(void) {
    UIApplication* app = UIApplication.sharedApplication;
    if (@available(iOS 13.0, *)) {
        UIWindowScene* fallback = nil;
        for (UIScene* scene in app.connectedScenes) {
            if (![scene isKindOfClass:UIWindowScene.class]) continue;
            if (!fallback) fallback = (UIWindowScene*)scene;
            if (scene.activationState == UISceneActivationStateForegroundActive)
                return (UIWindowScene*)scene;
        }
        return fallback;
    }
    return nil;
}

static PoolOverlayWindow* gPoolOverlayWindow = nil;
static __weak PoolOverlayView* gPoolOverlayView = nil;

static void PoolLabDestroyOverlay(void) {
    PoolOverlayView* overlay = gPoolOverlayView;
    [overlay shutdown];
    gPoolOverlayWindow.hidden = YES;
    gPoolOverlayWindow.rootViewController = nil;
    gPoolOverlayWindow = nil;
    gPoolOverlayView = nil;
}

static void PoolLabInstallOverlay(NSUInteger attempt) {
    UIWindowScene* scene = PoolLabFindScene();
    if (gPoolOverlayWindow) {
        if (@available(iOS 13.0, *)) {
            if (!scene || gPoolOverlayWindow.windowScene != scene) {
                PoolLabDestroyOverlay();
            }
        }
    }
    if (gPoolOverlayWindow) {
        gPoolOverlayWindow.hidden = NO;
        gPoolOverlayWindow.windowLevel = UIWindowLevelAlert + 1000.0;
        [gPoolOverlayView resumeSampling];
        return;
    }

    CGRect frame = UIScreen.mainScreen.bounds;
    BOOL sceneMissing = NO;
    if (@available(iOS 13.0, *)) {
        if (scene) frame = scene.coordinateSpace.bounds;
        else sceneMissing = YES;
    }
    if (CGRectIsEmpty(frame) || sceneMissing) {
        if (attempt < 60) {
            dispatch_after(dispatch_time(DISPATCH_TIME_NOW, (int64_t)(0.5 * NSEC_PER_SEC)),
                           dispatch_get_main_queue(), ^{ PoolLabInstallOverlay(attempt + 1); });
        }
        return;
    }

    PoolOverlayWindow* window = nil;
    if (@available(iOS 13.0, *)) {
        window = [[PoolOverlayWindow alloc] initWithWindowScene:scene];
        window.frame = frame;
    } else {
        window = [[PoolOverlayWindow alloc] initWithFrame:frame];
    }
    window.backgroundColor = UIColor.clearColor;
    window.opaque = NO;
    window.windowLevel = UIWindowLevelAlert + 1000.0;

    UIViewController* controller = [UIViewController new];
    controller.view.frame = window.bounds;
    controller.view.autoresizingMask = UIViewAutoresizingFlexibleWidth | UIViewAutoresizingFlexibleHeight;
    controller.view.backgroundColor = UIColor.clearColor;
    PoolOverlayView* overlay = [[PoolOverlayView alloc] initWithFrame:controller.view.bounds];
    [controller.view addSubview:overlay];
    window.rootViewController = controller;

    gPoolOverlayWindow = window;
    gPoolOverlayView = overlay;
    gPoolOverlayWindow.hidden = NO;  // Do not make key: game input remains on Unity's window.
}

__attribute__((constructor)) static void PoolTrajectoryHybridEntry(void) {
    dispatch_async(dispatch_get_main_queue(), ^{
        NSNotificationCenter* center = NSNotificationCenter.defaultCenter;
        [center addObserverForName:UIApplicationWillResignActiveNotification
                           object:nil queue:NSOperationQueue.mainQueue
                       usingBlock:^(__unused NSNotification* note) {
            [gPoolOverlayView pauseSampling];
        }];
        [center addObserverForName:UIApplicationDidEnterBackgroundNotification
                           object:nil queue:NSOperationQueue.mainQueue
                       usingBlock:^(__unused NSNotification* note) {
            [gPoolOverlayView pauseSampling];
        }];
        [center addObserverForName:UIApplicationDidBecomeActiveNotification
                           object:nil queue:NSOperationQueue.mainQueue
                       usingBlock:^(__unused NSNotification* note) {
            PoolLabInstallOverlay(0);
        }];
        [center addObserverForName:UIApplicationWillTerminateNotification
                           object:nil queue:NSOperationQueue.mainQueue
                       usingBlock:^(__unused NSNotification* note) {
            PoolLabDestroyOverlay();
        }];
        if (@available(iOS 13.0, *)) {
            [center addObserverForName:UISceneDidDisconnectNotification
                               object:nil queue:NSOperationQueue.mainQueue
                           usingBlock:^(NSNotification* note) {
                if (gPoolOverlayWindow.windowScene == note.object) PoolLabDestroyOverlay();
            }];
        }
        dispatch_after(dispatch_time(DISPATCH_TIME_NOW, (int64_t)(1.5 * NSEC_PER_SEC)),
                       dispatch_get_main_queue(), ^{ PoolLabInstallOverlay(0); });
    });
}
