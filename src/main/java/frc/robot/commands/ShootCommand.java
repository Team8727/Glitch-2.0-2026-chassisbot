package frc.robot.commands;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.shooting.ShootingRegressions;
import frc.robot.shooting.Targeting;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Shooter.ShooterControlMode;

/**
 * Adaptive shoot command with at-speed gating.
 *
 * <p>Spins up the flywheel, waits until it reaches target RPM (within a
 * dashboard-tunable tolerance), then feeds balls through the indexer.
 * The shooter stays running the entire time so velocity doesn't drop.
 *
 * <p>Tune from dashboard:
 * <ul>
 *   <li>{@code /SmartDashboard/Shooter RPM Tolerance} — default 50 RPM</li>
 *   <li>{@code /SmartDashboard/Shooter Feed Time} — how long to feed (default 1.0s)</li>
 * </ul>
 */
public class ShootCommand extends SequentialCommandGroup {

    public ShootCommand(Shooter shooter, Indexer indexer, Drivetrain drivetrain) {
        // Dashboard-tunable parameters
        SmartDashboard.putNumber("Shooter RPM Tolerance", 50.0);
        SmartDashboard.putNumber("Shooter Feed Time", Constants.kShooterFeedTime);

        addCommands(
                // Step 1: Spin up flywheel
                shooter.run(() -> {
                    var pose = drivetrain.getState().Pose;
                    double distance = Targeting.getHorizontalDistance(pose);
                    double motorRPS = ShootingRegressions.getMotorRPS(distance);
                    shooter.setVelocity(motorRPS, ShooterControlMode.FF_AND_PID);
                }),

                // Step 2: Wait until at speed (with timeout)
                new WaitForFlywheelSpeed(shooter, drivetrain),

                // Step 3: Feed balls while keeping shooter running
                parallel(
                        shooter.run(() -> {}), // keeps shooter required/active
                        indexer.run(() -> indexer.setDutyCycle(
                                Constants.kShooterIndexerDutyCycle))
                                .withTimeout(Constants.kShooterFeedTime))
        );
    }
}

/**
 * Waits until flywheel reaches target RPM within tolerance, or times out.
 * Dashboard-tunable tolerance via "Shooter RPM Tolerance".
 */
class WaitForFlywheelSpeed extends Command {
    private final Shooter shooter;
    private final Drivetrain drivetrain;
    private double startTime;

    private static final double kMaxWaitTime = Constants.kShooterSpinupTime + 2.0;

    public WaitForFlywheelSpeed(Shooter shooter, Drivetrain drivetrain) {
        this.shooter = shooter;
        this.drivetrain = drivetrain;
    }

    @Override
    public void initialize() {
        startTime = Timer.getFPGATimestamp();
    }

    @Override
    public boolean isFinished() {
        double tolerance = SmartDashboard.getNumber("Shooter RPM Tolerance", 50.0);
        if (tolerance <= 1.0) tolerance = 50.0;

        var pose = drivetrain.getState().Pose;
        double distance = Targeting.getHorizontalDistance(pose);
        double targetRPM = ShootingRegressions.getRPM(distance);
        double currentRPM = shooter.getFlywheelVelocity() * 60.0;

        boolean atSpeed = Math.abs(currentRPM - targetRPM) <= tolerance;
        boolean timedOut = (Timer.getFPGATimestamp() - startTime) > kMaxWaitTime;

        if (atSpeed || timedOut) {
            SmartDashboard.putNumber("Shooter RPM Target", targetRPM);
            SmartDashboard.putNumber("Shooter RPM Actual", currentRPM);
            return true;
        }
        return false;
    }
}