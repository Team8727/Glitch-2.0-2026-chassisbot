package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.shooting.Targeting;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.TunerConstants;

public class DriveCommands {
    private static final double kMaxSpeed =
            TunerConstants.kSpeedAt12Volts.in(edu.wpi.first.units.Units.MetersPerSecond)
                    * Constants.kTeleopSpeedFraction;
    private static final double kMaxAngularRate = TunerConstants.kMaxAngularVelocity;

    public static final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(kMaxSpeed * 0.1)
            .withRotationalDeadband(kMaxAngularRate * 0.1)
            .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage);

    private static final SwerveRequest.FieldCentricFacingAngle faceTarget =
            new SwerveRequest.FieldCentricFacingAngle()
                    .withDeadband(kMaxSpeed * 0.1)
                    .withRotationalDeadband(kMaxAngularRate * 0.75)
                    .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage)
                    .withHeadingPID(
                            Robot.isReal() ? Constants.kRotationKP : 50.0,
                            0,
                            Robot.isReal() ? Constants.kRotationKDReal : Constants.kRotationKDSim);

    private DriveCommands() {}

    /** Initialize dashboard tuning sliders for drive axis inversion (called once at robot boot). */
    public static void initDashboardTuning() {
        SmartDashboard.putNumber("Drive/Invert X", 1.0);   // -1 to invert forward
        SmartDashboard.putNumber("Drive/Invert Y", 1.0);   // -1 to invert strafe
        SmartDashboard.putNumber("Drive/Invert Rot", 1.0); // -1 to invert rotation
    }

    /** Read the SmartDashboard multiplier for each axis (1 = normal, -1 = inverted). */
    private static double invertX() { return SmartDashboard.getNumber("Drive/Invert X", 1.0); }
    private static double invertY() { return SmartDashboard.getNumber("Drive/Invert Y", 1.0); }
    private static double invertRot() { return SmartDashboard.getNumber("Drive/Invert Rot", 1.0); }

    /**
     * Default field-centric teleop drive command.
     *
     * @param drivetrain the swerve drivetrain
     * @param controller the driver Xbox controller
     * @return a command that drives field-centrically
     */
    public static Command teleopDrive(Drivetrain drivetrain, CommandXboxController controller) {
        return drivetrain.applyRequest(
                () ->
                        drive.withVelocityX(controller.getLeftY() * kMaxSpeed * invertX())
                                .withVelocityY(controller.getLeftX() * kMaxSpeed * invertY())
                                .withRotationalRate(-controller.getRightX() * kMaxAngularRate * invertRot()));
    }

    /**
     * Field-centric drive that automatically rotates to face the HUB.
     *
     * @param drivetrain the swerve drivetrain
     * @param controller the driver Xbox controller
     * @return a command that drives while auto-aiming at the HUB
     */
    public static Command autoAimDrive(Drivetrain drivetrain, CommandXboxController controller) {
        return drivetrain.applyRequest(
                () -> {
                    Pose2d robotPose = drivetrain.getState().Pose;
                    Rotation2d targetYaw = Targeting.computeYaw(robotPose);
                    return faceTarget
                            .withTargetDirection(targetYaw)
                            .withVelocityX(controller.getLeftY() * kMaxSpeed * invertX())
                            .withVelocityY(controller.getLeftX() * kMaxSpeed * invertY())
                            .withRotationalDeadband(kMaxAngularRate * 0.1);
                });
    }

    /**
     * Field-centric drive with an oscillation/wiggle applied to the target heading.
     *
     * @param drivetrain the swerve drivetrain
     * @param controller the driver Xbox controller
     * @return a command that drives with oscillating heading
     */
    public static Command oscillateDrive(Drivetrain drivetrain, CommandXboxController controller) {
        return drivetrain.applyRequest(
                () -> {
                    Pose2d robotPose = drivetrain.getState().Pose;
                    Rotation2d baseYaw = Targeting.computeYaw(robotPose);
                    double oscDeg = computeOscillation(2, 5);
                    return faceTarget
                            .withTargetDirection(
                                    Rotation2d.fromDegrees(
                                            baseYaw.getDegrees() - oscDeg))
                            .withVelocityX(controller.getLeftY() * kMaxSpeed * invertX())
                            .withVelocityY(controller.getLeftX() * kMaxSpeed * invertY())
                            .withRotationalDeadband(kMaxAngularRate * 0.1);
                });
    }

    /**
     * Registers the idle brake command to engage when the robot is disabled.
     *
     * @param drivetrain the swerve drivetrain
     */
    public static void registerIdleBrake(Drivetrain drivetrain) {
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled()
                .whileTrue(drivetrain.applyRequest(() -> idle).ignoringDisable(true));
    }

    // ========================================================================
    // SysId Commands
    // ========================================================================

    public static Command sysIdQuasistatic(Drivetrain drivetrain, SysIdRoutine.Direction dir) {
        return drivetrain.sysIdQuasistatic(dir);
    }

    public static Command sysIdDynamic(Drivetrain drivetrain, SysIdRoutine.Direction dir) {
        return drivetrain.sysIdDynamic(dir);
    }

    // ========================================================================
    // Helpers
    // ========================================================================

    /**
     * Computes a sinusoidal oscillation for wiggle drive.
     *
     * @param amplitudeDeg   peak oscillation amplitude in degrees
     * @param flipsPerSecond oscillation frequency
     * @return the current oscillation offset in degrees
     */
    private static double computeOscillation(double amplitudeDeg, double flipsPerSecond) {
        double timeSeconds = RobotController.getFPGATime() * 1e-6;
        double phase = 2.0 * Math.PI * flipsPerSecond * timeSeconds;
        return amplitudeDeg * Math.sin(phase);
    }
}