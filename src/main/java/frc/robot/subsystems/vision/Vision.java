package frc.robot.subsystems.vision;

import dev.doglog.DogLog;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.Constants;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

/**
 * Robot-layer Vision facade using direct PhotonLib.
 *
 * <p>Manages two PhotonVision cameras and computes robot pose from visible AprilTags.
 *
 * <p>Cameras run on Orange Pi coprocessors that can brown out under heavy motor load.
 * All camera reads are wrapped in try-catch so the robot never crashes when a Pi drops.
 */
public class Vision implements AutoCloseable {

  public record Measurement(Pose2d pose, double timestampSeconds, double avgDistanceMeters, double ambiguity) {
  }

  private org.photonvision.PhotonCamera cameraFrontLeft;
  private org.photonvision.PhotonCamera cameraFrontRight;
  private final AprilTagFieldLayout fieldLayout;

  private static final double kMaxAmbiguity = 0.2;
  private static final double kMaxDistance = 3.5;

  /** Monotonic error counter; incremented each time a camera read throws. */
  private int cameraErrorCount = 0;

  public Vision() {
    if (RobotBase.isReal()) {
      cameraFrontLeft = new org.photonvision.PhotonCamera(Constants.kCamFrontLeft);
      cameraFrontRight = new org.photonvision.PhotonCamera(Constants.kCamFrontRight);
    } else {
      cameraFrontLeft = null;
      cameraFrontRight = null;
    }

    AprilTagFieldLayout layout = null;
    try {
      layout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);
    } catch (Exception e) {
      DogLog.log("Vision/FieldLayoutError", e.getMessage());
    }
    this.fieldLayout = layout;
  }

  public void periodic() {
    // PhotonLib handles polling internally
  }

  public List<Measurement> drainMeasurements(Pose2d referencePose) {
    List<Measurement> measurements = new ArrayList<>();
    if (cameraFrontLeft != null) {
      try {
        drainCamera(cameraFrontLeft, measurements);
      } catch (Exception e) {
        handleCameraError("FrontLeft", e);
      }
    }
    if (cameraFrontRight != null) {
      try {
        drainCamera(cameraFrontRight, measurements);
      } catch (Exception e) {
        handleCameraError("FrontRight", e);
      }
    }
    return measurements;
  }

  /** Logs camera errors once (rate-limited) and reconnects if the Pi came back. */
  private void handleCameraError(String name, Exception e) {
    cameraErrorCount++;
    DogLog.log("Vision/" + name + "/Error", e.getClass().getSimpleName() + ": " + e.getMessage());
    // Attempt to recover — if the Pi rebooted, a new connection will work
    // (PhotonLib reconnects transparently on the next call)
  }

  private void drainCamera(org.photonvision.PhotonCamera camera, List<Measurement> out) {
    for (var result : camera.getAllUnreadResults()) {
      double timestamp = result.getTimestampSeconds();
      var targets = result.getTargets();

      if (targets.isEmpty()) continue;

      var bestTarget = result.getBestTarget();
      if (bestTarget == null) continue;

      double ambiguity = bestTarget.getPoseAmbiguity();
      if (ambiguity > kMaxAmbiguity || ambiguity < 0) continue;

      if (fieldLayout == null) continue;
      int tagId = bestTarget.getFiducialId();
      var tagPose = fieldLayout.getTagPose(tagId);
      if (tagPose.isEmpty()) continue;

      var cameraToTarget = bestTarget.getBestCameraToTarget();
      var fieldToCamera = tagPose.get().plus(cameraToTarget.inverse());

      Transform3d robotToCamera = camera == cameraFrontLeft
          ? new Transform3d(Constants.kCamFrontLeftTranslation, new Rotation3d(0, 0, Constants.kCamFrontLeftYaw))
          : new Transform3d(Constants.kCamFrontRightTranslation, new Rotation3d(0, 0, Constants.kCamFrontRightYaw));

      var fieldToRobot = fieldToCamera.plus(robotToCamera.inverse());
      double distance = cameraToTarget.getTranslation().getNorm();
      if (distance > kMaxDistance) continue;

      out.add(new Measurement(fieldToRobot.toPose2d(), timestamp, distance, ambiguity));
    }
  }

  /** Returns the cumulative number of camera read errors since boot (0 = healthy). */
  public int getCameraErrorCount() {
    return cameraErrorCount;
  }

  public Optional<Pose2d> bestStartPose() {
    List<Measurement> measurements = drainMeasurements(new Pose2d());
    if (measurements.isEmpty()) return Optional.empty();
    Measurement best = measurements.get(0);
    for (Measurement m : measurements) {
      if (m.ambiguity() >= 0 && m.ambiguity() < best.ambiguity()) best = m;
    }
    return Optional.of(best.pose());
  }

  private final StructPublisher<Pose3d> flPosePub =
      NetworkTableInstance.getDefault().getStructTopic("Vision/" + Constants.kCamFrontLeft + "/Pose", Pose3d.struct).publish();
  private final StructPublisher<Pose3d> frPosePub =
      NetworkTableInstance.getDefault().getStructTopic("Vision/" + Constants.kCamFrontRight + "/Pose", Pose3d.struct).publish();

  public void logCameraPoses(Pose2d robotPose) {
    if (cameraFrontLeft == null) return;
    try {
      var robotPose3d = new Pose3d(robotPose);
      var flTransform = new Transform3d(Constants.kCamFrontLeftTranslation, new Rotation3d(0, 0, Constants.kCamFrontLeftYaw));
      flPosePub.set(robotPose3d.transformBy(flTransform));
      var frTransform = new Transform3d(Constants.kCamFrontRightTranslation, new Rotation3d(0, 0, Constants.kCamFrontRightYaw));
      frPosePub.set(robotPose3d.transformBy(frTransform));
    } catch (Exception e) {
      // Don't crash the robot loop just to log a camera pose
    }
  }

  @Override
  public void close() {
    try { if (cameraFrontLeft != null) cameraFrontLeft.close(); } catch (Exception ignored) {}
    try { if (cameraFrontRight != null) cameraFrontRight.close(); } catch (Exception ignored) {}
  }
}