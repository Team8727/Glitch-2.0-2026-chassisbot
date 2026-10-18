package frc.robot.commands;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.shooting.ShootingRegressions;
import frc.robot.shooting.Targeting;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Shooter.ShooterControlMode;

/**
 * Adaptive shoot command — runs shooter + indexer as long as RT is held.
 *
 * <p>Spins up flywheel → waits until at speed → feeds balls continuously
 * until the trigger is released. The {@code whileTrue} binding in Controls
 * cancels this command on trigger release, which triggers the finallyDo
 * cleanup and stops both mechanisms.
 *
 * <p>Dashboard tuning:
 * <ul>
 *   <li>{@code /SmartDashboard/Shooter RPM Tolerance} — default 50 RPM</li>
 * </ul>
 */
public class ShootCommand extends SequentialCommandGroup {

    public ShootCommand(Shooter shooter, Indexer indexer, Drivetrain drivetrain) {
        SmartDashboard.putNumber("Shooter RPM Tolerance", 50.0);

        addCommands(
                sequence(
                        // Step 1: Spin up flywheel based on distance regression
                        shooter.run(() -> {
                            var pose = drivetrain.getState().Pose;
                            double distance = Targeting.getHorizontalDistance(pose);
                            double motorRPS = ShootingRegressions.getMotorRPS(distance);
                            shooter.setVelocity(motorRPS, ShooterControlMode.FF_AND_PID);
                        }),

                        // Step 2: Wait until flywheel reaches target speed
                        new WaitForFlywheelSpeed(shooter, drivetrain),

                        // Step 3: Feed balls continuously while keeping shooter running
                        // Both run until RT is released and the command is canceled
                        parallel(
                                shooter.run(() -> {}),
                                indexer.run(() -> indexer.setDutyCycle(
                                        Constants.kShooterIndexerDutyCycle)))
                ).finallyDo(() -> {
                    // Cleanup when trigger is released or command ends
                    shooter.setDutyCycle(0);
                    indexer.setDutyCycle(0);
                }));
    }
}

/**
 * Waits until flywheel reaches target RPM within tolerance, or times out.
 * Tolerance is dashboard-tunable via "Shooter RPM Tolerance" (default 50 RPM).
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