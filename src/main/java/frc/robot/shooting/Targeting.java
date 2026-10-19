package frc.robot.shooting;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.shooter.ShooterConstants;

/**
 * Targeting utilities: HUB aiming, zone detection, look-ahead projection.
 */
public final class Targeting {
    private Targeting() {}

    /** Field zones relative to the current alliance. */
    public enum Zone {
        ALLIANCE,  // Behind the bumps on your side
        NEUTRAL,   // Center of the field
        REMOTE     // Opponent's side
    }

    // ========================================================================
    // HUB targeting
    // ========================================================================

    public static Translation3d getTargetTranslation() {
        return Robot.isRedAlliance() ? Constants.kRedHubTarget : Constants.kBlueHubTarget;
    }

    public static Rotation2d computeYaw(Pose2d robotPose) {
        Translation3d target = getTargetTranslation();
        double dx = target.getX() - robotPose.getX();
        double dy = target.getY() - robotPose.getY();
        return new Rotation2d(dx, dy);
    }

    public static double getHorizontalDistance(Pose2d robotPose) {
        Translation3d target = getTargetTranslation();
        double dx = target.getX() - robotPose.getX();
        double dy = target.getY() - robotPose.getY();
        return Math.sqrt(dx * dx + dy * dy);
    }

    public static boolean isWithinRange(Pose2d robotPose) {
        double d = getHorizontalDistance(robotPose);
        return d >= ShooterConstants.kMinShootDistance
            && d <= ShooterConstants.kMaxShootDistance;
    }

    // ========================================================================
    // Look-ahead: project pose forward by estimated time-of-flight
    // ========================================================================

    /**
     * Returns a pose projected forward from the current pose and velocity,
     * using an estimated time-of-flight. This lets the robot aim at where it
     * WILL be when the ball lands, rather than where it is right now.
     *
     * @param pose   current robot pose
     * @param speeds current chassis speeds
     * @return projected future pose
     */
    public static Pose2d computeFuturePose(Pose2d pose, ChassisSpeeds speeds) {
        // Estimate time-of-flight from distance
        double dist = getHorizontalDistance(pose);
        double tof = estimateTimeOfFlight(dist);

        // Project forward using robot-relative velocity
        Twist2d twist = new Twist2d(
            speeds.vxMetersPerSecond * tof,
            speeds.vyMetersPerSecond * tof,
            speeds.omegaRadiansPerSecond * tof);
        return pose.exp(twist);
    }

    /**
     * Rough time-of-flight estimate for a given distance.
     * Uses a muzzle velocity estimate of ~8 m/s with 60° pitch.
     */
    private static double estimateTimeOfFlight(double distance) {
        // v ≈ 8 m/s typical, cos(60°) = 0.5 → horizontal speed ≈ 4 m/s
        double horizontalSpeed = 4.0;
        double tof = distance / horizontalSpeed;
        // Clamp to reasonable range
        if (tof < 0.15) tof = 0.15;
        if (tof > 1.0) tof = 1.0;
        return tof;
    }

    /**
     * Returns the look-ahead distance to the HUB, accounting for robot motion.
     * Uses the future projected pose instead of the current pose.
     */
    public static double getLookaheadDistance(Pose2d pose, ChassisSpeeds speeds) {
        Pose2d futurePose = computeFuturePose(pose, speeds);
        return getHorizontalDistance(futurePose);
    }

    /**
     * Returns the look-ahead yaw to the HUB.
     */
    public static Rotation2d computeLookaheadYaw(Pose2d pose, ChassisSpeeds speeds) {
        Pose2d futurePose = computeFuturePose(pose, speeds);
        return computeYaw(futurePose);
    }

    // ========================================================================
    // Zone awareness
    // ========================================================================

    /**
     * Returns which zone the robot is in, relative to the current alliance.
     * ALLIANCE = behind the bumps on your side
     * NEUTRAL  = center field
     * REMOTE   = opponent's side
     */
    public static Zone getZone(Pose2d robotPose) {
        double x = robotPose.getX();
        // Alliance-relative zone thresholds
        if (Robot.isRedAlliance()) {
            if (x > 2.5)  return Zone.ALLIANCE;
            if (x < -2.5) return Zone.REMOTE;
            return Zone.NEUTRAL;
        } else {
            if (x < -2.5) return Zone.ALLIANCE;
            if (x > 2.5)  return Zone.REMOTE;
            return Zone.NEUTRAL;
        }
    }
}