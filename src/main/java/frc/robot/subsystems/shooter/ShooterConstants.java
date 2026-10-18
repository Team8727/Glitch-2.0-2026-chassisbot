package frc.robot.subsystems.shooter;

/** Shooter-specific constants and regression tables. */
public final class ShooterConstants {
    private ShooterConstants() {}
    
    /** Distance (m) to flywheel speed (RPM) regression table for fixed-pitch shooter.
     *  The shooter has a fixed 60° pitch. These values map horizontal distance
     *  to the flywheel RPM needed to score in the HUB (center at 1.83m height). */
    public static final double[][] kDistanceToRPM = {
        {1.5,  1800},
        {2.0,  2000},
        {2.5,  2200},
        {3.0,  2400},
        {3.5,  2600},
        {4.0,  2800},
        {4.5,  3000},
        {5.0,  3200},
        {5.5,  3400},
        {6.0,  3600},
        {7.0,  3800},
    };
    
    /** Maximum distance we can reliably shoot (m). */
    public static final double kMaxShootDistance = 7.0;
    
    /** Minimum distance to attempt a shot (m). */
    public static final double kMinShootDistance = 1.0;
    
    /** RPM tolerance for "at speed" detection. */
    public static final double kAtSpeedToleranceRPM = 100.0;
    
    /** Alignment tolerance for shooting (degrees). */
    public static final double kAlignmentToleranceDeg = 3.0;
    
    /** Distance threshold for close-range shots (m). */
    public static final double kCloseRangeThreshold = 2.0;
    
    /** Distance threshold for long-range shots (m). */
    public static final double kLongRangeThreshold = 5.0;
    
    // ========== Indexer Config ==========
    
    /** Indexer velocity during shooting (RPM). */
    public static final double kIndexerShootSpeedRPM = 3000;
    
    /** Indexer kP for velocity control during shooting. */
    public static final double kIndexerShootKP = 0.07;
    
    /** Indexer current limit during shooting (A). */
    public static final int kIndexerShootCurrentLimit = 80;
}