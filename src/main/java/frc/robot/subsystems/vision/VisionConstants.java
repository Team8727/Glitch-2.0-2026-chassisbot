package frc.robot.subsystems.vision;

/** Constants for vision simulation configuration. */
public final class VisionConstants {
    private VisionConstants() {}

    /** Simulated camera image width (pixels). */
    public static final int kSimWidth = 640;

    /** Simulated camera image height (pixels). */
    public static final int kSimHeight = 480;

    /** Simulated camera field of view (degrees). */
    public static final double kSimFovDeg = 70;

    /** Simulated camera frames per second. */
    public static final int kSimFps = 40;

    /** Simulated camera average latency (ms). */
    public static final double kSimAvgLatMs = 35;

    /** Simulated camera latency standard deviation (ms). */
    public static final double kSimLatStdMs = 5;
}