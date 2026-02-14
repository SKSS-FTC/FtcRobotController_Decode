# TEAMCODE MODULE

**Generated:** 2026-02-11

## OVERVIEW
Team workspace for FTC competition code. Custom OpModes, hardware абстракции, and team-specific logic.

## STRUCTURE
```
TeamCode/src/main/java/org/firstinspires/ftc/teamcode/
├── autonomous/  # Autonomous OpModes
├── opmode/      # TeleOp OpModes
└── subsystem/   # Hardware subsystems (single responsibility)
```

## WHERE TO LOOK
| Task | Location | Notes |
|------|----------|-------|
| Auto OpModes | autonomous/ | Extend LinearOpMode or OpMode |
| TeleOp OpModes | opmode/ | Annotation: @TeleOp or @Autonomous |
| Subsystems | subsystem/ | Encapsulate hardware abstractions |
| Constants | subsystem/ | e.g., TransformationConstants.java for vision |

## CONVENTIONS
- Copy samples from ../FtcRobotController/external/samples/ to customize
- Follow Google Java Style Guide
- Device config names: snake_case matching FTC controller config
- Device object names: camelCase matching config names
- @TeleOp/@Autonomous required annotations for OpMode class
- group="Tests" for diagnostic/utility OpModes

## SUBSYSTEM MANAGEMENT (KISS + DRY)

### Current Subsystems
- `Camera.java` - VisionPortal lifecycle, camera selection (webcam/builtin), streaming control
- `YOLO26Processor.java` - TensorFlow inference, detection output, canvas drawing overlays
- `AprilTagReader.java` - AprilTag detection, pose estimation
- `Transformation.java` - Coordinate transformations, field geometry
- `Constants.java` - Shared constants

**Note:** Current components should be renamed to simple domain names: `Detection.java`, `AprilTag.java`

### Subsystem Rules

1. **Single Responsibility Per File**
   - Each subsystem handles ONE domain only
   - Camera manages camera ONLY
   - Detection does detection ONLY

2. **Call Other Subsystems**
   - If AprilTag needs camera, CALL Camera subsystem
   - DON'T implement camera logic in AprilTag yourself
   - Direct calls are fine - no interface complexity needed

3. **Builder Pattern for Complex Init**
   - See Camera.Builder for reference
   - Makes initialization clean and composable

4. **Lifecycle Methods**
   - `init()` for setup during OpMode init phase
   - `close()` for cleanup in OpMode stop phase
   - All resources properly managed

5. **Pure Getters**
   - Return state without modifying internals
   - `getDetections()` returns current detections list
   - No side effects during reads

### Example: Good Subsystem Pattern
```java
// GOOD: AprilTag CALLS Camera for what it needs
class AprilTag {
    private Camera camera;
    private AprilTagProcessor processor;

    void init() {
        camera = new Camera.Builder(cameraName).build();
        camera.startStreaming();
        // AprilTag doesn't manage VisionPortal directly
        processor = new AprilTagProcessor.Builder().build();
        camera.setProcessor(processor);
    }

    void detect() {
        // CALL Camera to get resource we need
        List<AprilTagDetection> detections = processor.getDetections();
        return detections;
    }
}
```

### Anti-Patterns (Avoid)
```java
// BAD: AprilTag implementing camera logic itself
class AprilTag {
    VisionPortal visionPortal;  // Camera already handles this!
    void initCamera() { ... }   // CALL Camera subsystem instead
}

// BAD: Camera doing detection work
class Camera {
    void detectObjects() { ... }  // belongs in Detection
    void findAprilTags() { ... }  // belongs in AprilTag
}

// BAD: Redundant naming
class YOLO26Processor { ... }    // Use Detection.java instead
class CameraManager { ... }       // Use Camera.java instead
class VisionHelper { ... }        // Wrong pattern - helpers belong to caller
```

### When NOT to Create Subsystems
- Single-use OpMode logic → keep in OpMode
- One-line wrapper functions → direct call is simpler
- Trivial configuration → use OpMode fields directly

## SAMPLE NAMING CONVENTIONS
See: `../FtcRobotController/external/samples/sample_conventions.md`
- Basic: OpMode skeleton/structure
- Sensor: Specific sensor usage ( Sensor-{Company}-{Type} )
- Robot: Drive base demos ( Robot-{Mode}-{Action}-{OpModeType} )
- Concept: Single function demos ( Concept-{Topic}-{OpModeType} )
- Utility: Dev tools, calibration

## TEST PATTERN
NO traditional unit testing. Test via OpModes on actual hardware. Use sample OpModes to verify sensor integration and robot behavior.
