package frc.robot;

import Glitch.Lib.NetworkTableLogger;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

import java.util.Arrays;
import java.util.List;
import java.util.Optional;
import java.util.stream.Collectors;

/**
 * Robot-layer Vision configuration and facade.
 * <p>
 * This class owns all robot-specific vision configuration (camera names, robot-to-camera transforms,
 * filter thresholds, and simulation properties) and builds a library-level Vision provider
 * for real or simulated environments. It presents a minimal API suitable for consumption by
 * the robot-layer pose subsystem.
 */
public class Vision implements AutoCloseable {
  /**
   * Robot-layer measurement wrapper to decouple robot code from library nested types.
   * This is a record, so Measurement automatically has accessor methods for pose and timestampSeconds.
   * <p>
   * Values are produced by the underlying library provider but re-wrapped here for stability
   * of the robot API.
   *
   * @param pose             Estimated field pose at capture time.
   * @param timestampSeconds Capture timestamp in seconds.
   */
  public record Measurement(Pose2d pose, double timestampSeconds) {
  }

  private final Glitch.Lib.Vision.Vision.Provider provider;
  private final NetworkTableLogger logger = new NetworkTableLogger("Vision");

  // Camera names as configured in PhotonVision
//  private static final String CAM_FRONT_RIGHT = "FrontRight";
//  private static final String CAM_FRONT_LEFT = "FrontLeft";
//  private static final String CAM_BACK_RIGHT = "BackRight";
//  private static final String CAM_BACK_LEFT = "BackLeft";

  private static final String CAM_BACK_RIGHT_FRONT = "BackRightFront";
  private static final String CAM_BACK_LEFT_FRONT = "BackLeftFront";
  private static final String CAM_BACK_RIGHT_BACK = "BackRightBack";
  private static final String CAM_BACK_LEFT_BACK = "BackLeftBack";


  // Robot-to-camera transforms for Chassis Bot
//  private static final Transform3d FRONT_RIGHT_POS =
//      new Transform3d(
//          new Translation3d(Units.inchesToMeters(10.25), Units.inchesToMeters(-9.75), Units.inchesToMeters(8.25)),
//          new Rotation3d(Math.toRadians(0), Math.toRadians(-15), Math.toRadians(-45))); // Camera has no roll (on camera plane), 15 degree pitch (degrees to point camera up relative to camera facing forward), yaw (rotation around z axis that points directly upward from the robot).
//  private static final Transform3d FRONT_LEFT_POS =
//    new Transform3d(
//        new Translation3d(Units.inchesToMeters(10.25), Units.inchesToMeters(9.75), Units.inchesToMeters(8.25)),
//        new Rotation3d(Math.toRadians(11), Math.toRadians(-12.77), Math.toRadians(99)));
//  private static final Transform3d BACK_RIGHT_POS =
//    new Transform3d(
//        new Translation3d(Units.inchesToMeters(-10.25), Units.inchesToMeters(-9.75), Units.inchesToMeters(8.25)),
//        new Rotation3d(Math.toRadians(0), Math.toRadians(-15), Math.toRadians(225)));
//  private static final Transform3d BACK_LEFT_POS =
//    new Transform3d(
//        new Translation3d(Units.inchesToMeters(-10.25), Units.inchesToMeters(9.75), Units.inchesToMeters(8.25)),
//        new Rotation3d(Math.toRadians(0), Math.toRadians(-15), Math.toRadians(135)));

  //New Cameras for Season Robot:
  private static final Transform3d BACK_LEFT_FRONT =
    new Transform3d(
        new Translation3d(Units.inchesToMeters(-9),Units.inchesToMeters(11),Units.inchesToMeters(8.25)), //Translation just for testing
        new Rotation3d(Math.toRadians(15),Math.toRadians(-17),Math.toRadians(75))
    );
  private static final Transform3d BACK_LEFT_BACK =
          new Transform3d(
                  new Translation3d(Units.inchesToMeters(-10.75),Units.inchesToMeters(10.25),Units.inchesToMeters(8.25)), //Translation just for testing
                  new Rotation3d(Math.toRadians(-15),Math.toRadians(-17),Math.toRadians(160))
          );
  private static final Transform3d BACK_RIGHT_FRONT =
          new Transform3d(
                  new Translation3d(Units.inchesToMeters(-9),Units.inchesToMeters(-11),Units.inchesToMeters(8.25)), //Translation just for testing
                  new Rotation3d(Math.toRadians(-15),Math.toRadians(-17),Math.toRadians(-75))
          );
  private static final Transform3d BACK_RIGHT_BACK =
          new Transform3d(
                  new Translation3d(Units.inchesToMeters(-10.75),Units.inchesToMeters(-10.25),Units.inchesToMeters(8.25)), //Translation just for testing
                  new Rotation3d(Math.toRadians(15),Math.toRadians(-17),Math.toRadians(-160))
          );

  // Thresholds and sim properties
  private static final double MAX_AMBIGUITY = 0.2; // ignore -1 (handled in provider)
  private static final double MAX_DISTANCE_METERS = 3.5;
  private static final int SIM_WIDTH = 640;
  private static final int SIM_HEIGHT = 480;
  private static final double SIM_FOV_DEG = 70; // TODO: Change if using 120 degree lenses (which we have in stock) on cameras!!!
  private static final int SIM_FPS = 40;
  private static final double SIM_AVG_LAT_MS = 35;
  private static final double SIM_LAT_STD_MS = 5;

  /**
   * Constructs the robot-layer Vision facade and underlying provider.
   * <p>
   * Chooses a real vs. simulated provider based on {@link Robot#isSimulation()} and injects
   * robot-specific camera configurations and thresholds.
   */
  public Vision() {
    List<Glitch.Lib.Vision.Vision.CameraConfig> cameras = Arrays.asList(
//        new Glitch.Lib.Vision.Vision.CameraConfig(CAM_FRONT_RIGHT, FRONT_RIGHT_POS),
//        new Glitch.Lib.Vision.Vision.CameraConfig(CAM_FRONT_LEFT, FRONT_LEFT_POS),
//        new Glitch.Lib.Vision.Vision.CameraConfig(CAM_BACK_RIGHT, BACK_RIGHT_POS),
//        new Glitch.Lib.Vision.Vision.CameraConfig(CAM_BACK_LEFT, BACK_LEFT_POS)
        new Glitch.Lib.Vision.Vision.CameraConfig(CAM_BACK_LEFT_BACK, BACK_LEFT_BACK),
        new Glitch.Lib.Vision.Vision.CameraConfig(CAM_BACK_LEFT_FRONT, BACK_LEFT_FRONT),
        new Glitch.Lib.Vision.Vision.CameraConfig(CAM_BACK_RIGHT_FRONT, BACK_RIGHT_FRONT),
        new Glitch.Lib.Vision.Vision.CameraConfig(CAM_BACK_RIGHT_BACK, BACK_RIGHT_BACK)
    );

    Glitch.Lib.Vision.Vision.Config cfg = new Glitch.Lib.Vision.Vision.Config(
            cameras,
            MAX_AMBIGUITY,
            MAX_DISTANCE_METERS,
            SIM_WIDTH, SIM_HEIGHT, SIM_FOV_DEG, SIM_FPS,
            SIM_AVG_LAT_MS, SIM_LAT_STD_MS);

    AprilTagFieldLayout layout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);

    if (Robot.isSimulation()) {
      this.provider = Glitch.Lib.Vision.Vision.createPhotonVisionSim(cfg, layout);
    } else {
      this.provider = Glitch.Lib.Vision.Vision.createPhotonVision(cfg, layout);
    }
  }

  /**
   * Optional provider hook for per-loop processing.
   */
  public void periodic() {
    provider.periodic();
  }

  /**
   * Retrieves and maps any available measurements from the provider using the supplied reference pose.
   *
   * @param referencePose Current best estimate of robot pose; used by the provider to disambiguate
   * @return Robot-layer measurements (pose, timestamp) suitable for the pose estimator
   */
  public List<Measurement> drainMeasurements(Pose2d referencePose) {
    return provider.drainMeasurements(referencePose)
            .stream()
            .map(m -> new Measurement(m.pose, m.timestampSeconds))
            .collect(Collectors.toList());
  }

  /**
   * Attempts to compute an initial robot pose from visible tags (e.g., at boot).
   *
   * @return Optional initial field pose if a valid target is currently visible
   */
  public Optional<Pose2d> bestStartPose() {
    return provider.bestStartPose();
  }

  /**
   * @return Optional debug {@link Field2d} provided by the underlying provider (sim only).
   */
  public Optional<Field2d> getDebugField() {
    return provider.getDebugField();
  }

  /**
   * Computes each camera's current field pose from the supplied robot pose and publishes
   * them to NetworkTables for visualization (e.g., AdvantageScope/Glass).
   *
   * @param robotPose Current robot field pose (fused or odometry)
   */
  public void logCameraPoses(Pose2d robotPose) {
    var robotPose3d = new edu.wpi.first.math.geometry.Pose3d(robotPose);

    // Chassis Bot Cams
//    var camFrontRight = robotPose3d.transformBy(FRONT_RIGHT_POS);
//    var camFrontLeft = robotPose3d.transformBy(FRONT_LEFT_POS);
//    var camBackRight = robotPose3d.transformBy(BACK_RIGHT_POS);
//    var camBackLeft = robotPose3d.transformBy(BACK_LEFT_POS);
//    logger.logPose3d("/" + CAM_FRONT_RIGHT + "/Pose", camFrontRight);
//    logger.logPose3d("/" + CAM_FRONT_LEFT + "/Pose", camFrontLeft);
//    logger.logPose3d("/" + CAM_BACK_RIGHT + "/Pose", camBackRight);
//    logger.logPose3d("/" + CAM_BACK_LEFT + "/Pose", camBackLeft);

    // Real Bot Cams
    var camBackLeftFront = robotPose3d.transformBy(BACK_LEFT_FRONT);
    logger.logPose3d("/" + CAM_BACK_LEFT_FRONT+ "/Pose", camBackLeftFront);
    var camBackRightFront = robotPose3d.transformBy(BACK_RIGHT_FRONT);
    logger.logPose3d("/" + CAM_BACK_RIGHT_FRONT + "/Pose", camBackRightFront);
    var camBackLeftBack = robotPose3d.transformBy(BACK_LEFT_BACK);
    logger.logPose3d("/" + CAM_BACK_LEFT_BACK + "/Pose", camBackLeftBack);
    var camBackRightBack = robotPose3d.transformBy(BACK_RIGHT_BACK);
    logger.logPose3d("/" + CAM_BACK_RIGHT_BACK + "/Pose", camBackRightBack);


  }

  /**
   * Releases any provider resources (no-op by default).
   */
  @Override
  public void close() {
    provider.close();
  }
}
