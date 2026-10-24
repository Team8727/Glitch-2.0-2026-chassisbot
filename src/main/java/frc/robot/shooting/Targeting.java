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
 * Targeting utilities: HUB aiming, zone detection, look-ahead projection,
 * and selectable aim targets cycled by the driver.
 *
 * <p>Aim Targets (switched with Right Bumper):
 * <ol>
 *   <li><b>HUB</b> — Score position. Heads straight for the alliance HUB
 *       (AprilTags 25 &amp; 26). Default.</li>
 *   <li><b>NEUTRAL_PASS</b> — Aim into the neutral zone to pass the note
 *       across the field to a teammate.</li>
 *   <li><b>ALLIANCE</b> — Aim deeper into your own side, for short-range
 *       passing to a robot behind.</li>
 * </ol>
 */
public final class Targeting {
    private Targeting() {}

    // ========================================================================
    // Selectable aim targets (replaces the old 5° yaw offset)
    // ========================================================================

    /** Ordered list of aim targets the right bumper cycles through. */
    public enum AimTarget {
        HUB("HUB (25/26)"),
        NEUTRAL_PASS("Neutral Pass"),
        ALLIANCE("Alliance Pass");

        public final String label;
        AimTarget(String label) { this.label = label; }
    }

    private static AimTarget m_currentTarget = AimTarget.HUB;

    /** Cycles to the next aim target (wraps around). */
    public static void cycleTargetForward() {
        AimTarget[] values = AimTarget.values();
        m_currentTarget = values[(m_currentTarget.ordinal() + 1) % values.length];
    }

    /** Returns the currently selected aim target. */
    public static AimTarget getCurrentTarget() {
        return m_currentTarget;
    }

    /** Returns a human-readable name of the current target. */
    public static String getCurrentTargetName() {
        return m_currentTarget.label;
    }

    /** Resets the aim target back to HUB (called on disabled init). */
    public static void resetTarget() {
        m_currentTarget = AimTarget.HUB;
    }

    // ========================================================================
    // Target positions
    // ========================================================================

    /**
     * Returns the field position of the currently selected aim target,
     * mirrored for the current alliance colour.
     */
    public static Translation3d getTargetTranslation() {
        return switch (m_currentTarget) {
            case HUB -> Robot.isRedAlliance() ? Constants.kRedHubTarget : Constants.kBlueHubTarget;
            case NEUTRAL_PASS -> Robot.isRedAlliance() ? Constants.kRedNeutralPass : Constants.kBlueNeutralPass;
            case ALLIANCE -> Robot.isRedAlliance() ? Constants.kRedAllianceTarget : Constants.kBlueAllianceTarget;
        };
    }

    /** Returns the scoring HUB position for the current alliance (ignores any
     *  selected pass target). Used for the flywheel power and A-button auto-aim. */
    public static Translation3d getHubTargetTranslation() {
        return Robot.isRedAlliance() ? Constants.kRedHubTarget : Constants.kBlueHubTarget;
    }

    /** Field zones relative to the current alliance. */
    public enum Zone {
        ALLIANCE,  // Behind the bumps on your side
        NEUTRAL,   // Center of the field
        REMOTE     // Opponent's side
    }

    /** The shooter is mounted on the rear of the robot. Add 180° so the robot
     *  faces away from the target (intake forward), pointing the back at it. */
    private static final double kMountOffsetDeg = 180.0;

    /** Yaw toward the currently selected aim target (HUB / Neutral / Alliance). */
    public static Rotation2d computeYaw(Pose2d robotPose) {
        Translation3d target = getTargetTranslation();
        double dx = target.getX() - robotPose.getX();
        double dy = target.getY() - robotPose.getY();
        return Rotation2d.fromDegrees(
            Math.toDegrees(Math.atan2(dy, dx)) + kMountOffsetDeg);
    }

    /** Yaw toward the alliance HUB specifically — used by the A button auto-aim
     *  so it always locks onto the scoring target, not a pass position. */
    public static Rotation2d computeHubYaw(Pose2d robotPose) {
        Translation3d hub = Robot.isRedAlliance() ? Constants.kRedHubTarget : Constants.kBlueHubTarget;
        double dx = hub.getX() - robotPose.getX();
        double dy = hub.getY() - robotPose.getY();
        return Rotation2d.fromDegrees(
            Math.toDegrees(Math.atan2(dy, dx)) + kMountOffsetDeg);
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
     */
    public static Pose2d computeFuturePose(Pose2d pose, ChassisSpeeds speeds) {
        double dist = getHorizontalDistance(pose);
        double tof = estimateTimeOfFlight(dist);
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
        double horizontalSpeed = 4.0;
        double tof = distance / horizontalSpeed;
        if (tof < 0.15) tof = 0.15;
        if (tof > 1.0) tof = 1.0;
        return tof;
    }

    public static double getLookaheadDistance(Pose2d pose, ChassisSpeeds speeds) {
        Pose2d futurePose = computeFuturePose(pose, speeds);
        return getHorizontalDistance(futurePose);
    }

    public static Rotation2d computeLookaheadYaw(Pose2d pose, ChassisSpeeds speeds) {
        Pose2d futurePose = computeFuturePose(pose, speeds);
        return computeYaw(futurePose);
    }

    // ========================================================================
    // Zone awareness
    // ========================================================================

    public static Zone getZone(Pose2d robotPose) {
        double x = robotPose.getX();
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