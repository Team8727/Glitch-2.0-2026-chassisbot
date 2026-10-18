package frc.robot.shooting;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.shooter.ShooterConstants;

/**
 * Target selection and firing solution computation.
 *
 * <p>Provides static methods to compute the yaw angle and horizontal distance
 * from the robot to the HUB, handling alliance-based mirroring.
 */
public final class Targeting {
    private Targeting() {}

    /**
     * Returns the 3D target position of the HUB for the current alliance.
     *
     * @return the target Translation3d (meters)
     */
    public static Translation3d getTargetTranslation() {
        return Robot.isRedAlliance() ? Constants.kRedHubTarget : Constants.kBlueHubTarget;
    }

    /**
     * Computes the yaw angle required to face the HUB from the given robot pose.
     *
     * @param robotPose the current robot pose on the field
     * @return Rotation2d pointing toward the HUB
     */
    public static Rotation2d computeYaw(Pose2d robotPose) {
        Translation3d target = getTargetTranslation();
        double dx = target.getX() - robotPose.getX();
        double dy = target.getY() - robotPose.getY();
        return new Rotation2d(dx, dy);
    }

    /**
     * Computes the horizontal distance from the robot to the HUB.
     *
     * @param robotPose the current robot pose on the field
     * @return distance in meters
     */
    public static double getHorizontalDistance(Pose2d robotPose) {
        Translation3d target = getTargetTranslation();
        double dx = target.getX() - robotPose.getX();
        double dy = target.getY() - robotPose.getY();
        return Math.sqrt(dx * dx + dy * dy);
    }

    /**
     * Returns true if the robot is within the valid shooting range for the HUB.
     *
     * @param robotPose the current robot pose on the field
     * @return true if within min/max shoot distance
     */
    public static boolean isWithinRange(Pose2d robotPose) {
        double d = getHorizontalDistance(robotPose);
        return d >= ShooterConstants.kMinShootDistance
            && d <= ShooterConstants.kMaxShootDistance;
    }
}