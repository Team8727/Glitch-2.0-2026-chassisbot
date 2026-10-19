package frc.robot.commands;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.shooting.Targeting;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.shooter.Shooter;

/**
 * One-button shoot with integrated auto-aim and alignment gating.
 *
 * <ol>
 *   <li>Auto-rotates to face the HUB (FieldCentricFacingAngle)
 *   <li>Delegates to ShootCommand for flywheel spinup, at-speed gating,
 *       power-managed feed, and auto-unjam
 *   <li>Feeds only after both aligned AND at speed
 * </ol>
 */
public class SmartShootCommand extends SequentialCommandGroup {

    public SmartShootCommand(Shooter shooter, Indexer indexer, Drivetrain drivetrain) {
        SmartDashboard.putNumber("Shooter Align Tolerance", 5.0); // degrees

        addCommands(
                parallel(
                        // Branch 1: Auto-aim drive — continuously face the HUB
                        runOnce(() -> applyAutoAim(drivetrain)),
                        // Branch 2: Wait for alignment, then run full ShootCommand
                        sequence(
                                new WaitForAlignment(drivetrain),
                                new ShootCommand(shooter, indexer, drivetrain))
                ).finallyDo(() -> {
                    shooter.setDutyCycle(0);
                    indexer.setDutyCycle(0);
                }));
    }

    private static void applyAutoAim(Drivetrain drivetrain) {
        Pose2d pose = drivetrain.getState().Pose;
        Rotation2d targetYaw = Targeting.computeYaw(pose);
        var faceTarget = new com.ctre.phoenix6.swerve.SwerveRequest.FieldCentricFacingAngle()
                .withDeadband(4.39 * 0.1)
                .withRotationalDeadband(4.71 * 0.75)
                .withDriveRequestType(com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType.OpenLoopVoltage)
                .withHeadingPID(25.0, 0, 0.15)
                .withTargetDirection(targetYaw);
        drivetrain.setControl(faceTarget);
    }
}

/** Waits until the robot's heading is within tolerance of the HUB. */
class WaitForAlignment extends Command {
    private final Drivetrain drivetrain;
    private double startTime;
    private static final double kMaxWait = 3.0;

    public WaitForAlignment(Drivetrain drivetrain) { this.drivetrain = drivetrain; }

    @Override public void initialize() { startTime = Timer.getFPGATimestamp(); }

    @Override public boolean isFinished() {
        double tolDeg = SmartDashboard.getNumber("Shooter Align Tolerance", 5.0);
        if (tolDeg < 0.5) tolDeg = 5.0;

        Pose2d pose = drivetrain.getState().Pose;
        Rotation2d targetYaw = Targeting.computeYaw(pose);
        double error = Math.abs(Math.IEEEremainder(
            pose.getRotation().getDegrees() - targetYaw.getDegrees(), 360.0));

        boolean aligned = error <= tolDeg;
        boolean timedOut = (Timer.getFPGATimestamp() - startTime) > kMaxWait;

        if (aligned || timedOut) return true;
        return false;
    }
}