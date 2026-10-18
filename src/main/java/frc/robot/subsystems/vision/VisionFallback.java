package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;

/**
 * Dead-reckoning fallback for when vision cameras are not publishing.
 *
 * <p>When vision measurements stop arriving, this class takes over pose
 * estimation using only robot-relative odometry (swerve encoders + gyro).
 * It publishes a human-readable status so the dashboard can display whether
 * the pose is trustworthy.
 *
 * <p>Confidence levels:
 * <ul>
 *   <li>{@code VISION_OK} — recent vision measurement available, pose is accurate</li>
 *   <li>{@code DEAD_RECKONING} — no vision for >{@value #VISION_TIMEOUT}s,
 *       pose is drifting; caller should treat X/Y as approximate</li>
 *   <li>{@code STARTUP} — no pose known at all; robot just booted</li>
 * </ul>
 */
public class VisionFallback {
  public enum Confidence {
    /** Vision measurements arriving normally. Pose is accurate. */
    VISION_OK,
    /** No vision for more than the timeout. Pose is drifting. */
    DEAD_RECKONING,
    /** Unknown starting position. Pose should not be trusted. */
    STARTUP
  }

  /** How long (seconds) without a vision measurement before we switch to dead-reckoning. */
  private static final double VISION_TIMEOUT = 0.5;

  private Pose2d lastKnownPose = new Pose2d();
  private double lastVisionTimestamp = 0.0;
  private double lastUpdateTime = Timer.getFPGATimestamp();
  private Confidence confidence = Confidence.STARTUP;

  private final StringPublisher statusPub;

  public VisionFallback() {
    statusPub = NetworkTableInstance.getDefault()
        .getStringTopic("VisionFallback/status").publish();
    statusPub.set("STARTUP");
  }

  /**
   * Call every cycle from Drivetrain.periodic().
   *
   * @param visionCount     number of vision measurements fused this cycle (0 if none)
   * @param chassisSpeeds   current robot-relative chassis speeds
   * @param currentPose     the current fused pose from the drivetrain estimator
   * @return the pose to use (either vision-fused or dead-reckoned)
   */
  public Pose2d update(int visionCount, ChassisSpeeds chassisSpeeds, Pose2d currentPose) {
    double now = Timer.getFPGATimestamp();
    double dt = now - lastUpdateTime;
    lastUpdateTime = now;

    if (visionCount > 0) {
      // Vision is working — trust the fused pose
      lastKnownPose = currentPose;
      lastVisionTimestamp = now;
      confidence = Confidence.VISION_OK;
      publishStatus("VISION_OK");
      return currentPose;
    }

    // No vision this cycle — check timeout
    if (now - lastVisionTimestamp < VISION_TIMEOUT) {
      // Within grace period, keep using fused pose (it's still recent)
      confidence = Confidence.VISION_OK;
      publishStatus("VISION_OK");
      return currentPose;
    }

    // Vision is stale — dead-reckon from last known pose
    confidence = Confidence.DEAD_RECKONING;
    publishStatus("DEAD_RECKONING (" + String.format("%.1f", now - lastVisionTimestamp) + "s ago)");

    // Integrate robot-relative velocity into last known pose
    Twist2d twist = new Twist2d(
        chassisSpeeds.vxMetersPerSecond * dt,
        chassisSpeeds.vyMetersPerSecond * dt,
        chassisSpeeds.omegaRadiansPerSecond * dt);
    lastKnownPose = lastKnownPose.exp(twist);

    return lastKnownPose;
  }

  /**
   * Resets the fallback to a known pose (e.g. after seeing a tag or
   * being placed on the field).
   */
  public void reset(Pose2d knownPose) {
    lastKnownPose = knownPose;
    lastVisionTimestamp = Timer.getFPGATimestamp();
    confidence = Confidence.VISION_OK;
    publishStatus("VISION_OK (reset)");
  }

  /** Returns the current confidence level. */
  public Confidence getConfidence() {
    return confidence;
  }

  /** Returns true if pose is currently trustworthy. */
  public boolean isPoseTrustworthy() {
    return confidence == Confidence.VISION_OK;
  }

  private void publishStatus(String msg) {
    statusPub.set(msg);
  }
}