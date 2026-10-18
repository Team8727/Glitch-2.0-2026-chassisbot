package frc.robot.controls;

/** Constants for controller bindings and drive feel. */
public final class ControlsConstants {
    private ControlsConstants() {}

    /** Joystick deadband for translation inputs. */
    public static final double kTranslationDeadband = 0.1;

    /** Joystick deadband for rotation inputs. */
    public static final double kRotationDeadband = 0.1;

    /** Translation speed multiplier as a fraction of max speed. */
    public static final double kTranslationSpeedFactor = 0.8;

    /** Rotation speed multiplier as a fraction of max angular rate. */
    public static final double kRotationSpeedFactor = 0.8;

    /** Joystick magnitude below which auto-brake engages. */
    public static final double kAutoBrakeDeadband = 0.1;
}