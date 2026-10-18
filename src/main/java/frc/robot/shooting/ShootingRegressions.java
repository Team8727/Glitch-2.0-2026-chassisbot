package frc.robot.shooting;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import frc.robot.Constants;
import frc.robot.subsystems.shooter.ShooterConstants;

/**
 * Lookup table for distance-to-flywheel-RPM using linear interpolation.
 *
 * <p>Allows computing the required flywheel speed for any distance within
 * the range defined by {@link ShooterConstants#kDistanceToRPM}.
 */
public final class ShootingRegressions {
    private static final InterpolatingDoubleTreeMap distanceToRPM = new InterpolatingDoubleTreeMap();

    static {
        for (double[] entry : ShooterConstants.kDistanceToRPM) {
            distanceToRPM.put(entry[0], entry[1]);
        }
    }

    private ShootingRegressions() {}

    /**
     * Returns the interpolated flywheel RPM for the given horizontal distance.
     *
     * @param distanceMeters horizontal distance from robot to hub
     * @return target flywheel RPM
     */
    public static double getRPM(double distanceMeters) {
        if (distanceMeters <= 0) return distanceToRPM.get(0.1);
        if (distanceMeters > ShooterConstants.kMaxShootDistance) {
            return distanceToRPM.get(ShooterConstants.kMaxShootDistance);
        }
        return distanceToRPM.get(distanceMeters);
    }

    /**
     * Returns the flywheel speed in revolutions per second.
     *
     * @param distanceMeters horizontal distance from robot to hub
     * @return target flywheel RPS
     */
    public static double getRevolutionsPerSecond(double distanceMeters) {
        return getRPM(distanceMeters) / 60.0;
    }

    /**
     * Converts flywheel RPM to muzzle velocity (m/s) using flywheel diameter.
     *
     * @param rpm flywheel RPM
     * @return muzzle velocity in meters per second
     */
    public static double rpmToMuzzleVelocity(double rpm) {
        double rps = rpm / 60.0;
        return rps * Math.PI * Constants.kShooterFlywheelDiameterMeters;
    }

    /**
     * Gets the required motor speed in RPS for the given distance,
     * accounting for the flywheel gear ratio.
     *
     * @param distanceMeters horizontal distance from robot to hub
     * @return required motor RPS
     */
    public static double getMotorRPS(double distanceMeters) {
        double flywheelRPS = getRevolutionsPerSecond(distanceMeters);
        return flywheelRPS * Constants.kFlywheelGearing;
    }

    /**
     * Gets the target flywheel velocity for PID control in RPS,
     * accounting for the flywheel gear ratio.
     *
     * @param distanceMeters horizontal distance from robot to hub
     * @return target flywheel RPS for PID
     */
    public static double getFlywheelRPS(double distanceMeters) {
        return getRevolutionsPerSecond(distanceMeters) * Constants.kFlywheelGearing;
    }
}