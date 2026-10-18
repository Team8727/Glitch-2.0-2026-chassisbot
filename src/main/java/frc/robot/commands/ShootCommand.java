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
 * Adaptive shoot command with at-speed gating.
 *
 * <p>Spins up the flywheel, waits until it reaches target RPM (within a
 * dashboard-tunable tolerance), then feeds balls through the indexer.
 *
 * <p>The RPM tolerance can be tuned from the dashboard via
 * {@code /SmartDashboard/Shooter RPM Tolerance}.
 */
public class ShootCommand extends SequentialCommandGroup {

    public ShootCommand(Shooter shooter, Indexer indexer, Drivetrain drivetrain) {
        SmartDashboard.putNumber("Shooter RPM Tolerance", 100.0);

        addCommands(
                parallel(
                        // Spin up flywheel based on distance regression
                        shooter.run(() -> {
                            var pose = drivetrain.getState().Pose;
                            double distance = Targeting.getHorizontalDistance(pose);
                            double motorRPS = ShootingRegressions.getMotorRPS(distance);
                            shooter.setVelocity(motorRPS, ShooterControlMode.FF_AND_PID);
                        }),

                        // Wait for flywheel to reach speed, then feed
                        sequence(
                                new WaitForFlywheelSpeed(shooter, drivetrain),
                                indexer.run(() -> indexer.setDutyCycle(
                                        Constants.kShooterIndexerDutyCycle))
                                        .withTimeout(Constants.kShooterFeedTime)))
                .finallyDo(() -> {
                    shooter.setDutyCycle(0);
                    indexer.setDutyCycle(0);
                }));
    }
}

/**
 * Waits until the flywheel reaches target RPM within tolerance, or times out.
 * Tune tolerance from dashboard via "Shooter RPM Tolerance" (default 100 RPM).
 */
class WaitForFlywheelSpeed extends Command {
    private final Shooter shooter;
    private final Drivetrain drivetrain;
    private double startTime;

    private static final double kMaxWaitTime = Constants.kShooterSpinupTime + 1.0;

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
        double tolerance = SmartDashboard.getNumber("Shooter RPM Tolerance", 100.0);
        if (tolerance <= 0) tolerance = 100.0;

        var pose = drivetrain.getState().Pose;
        double distance = Targeting.getHorizontalDistance(pose);
        double targetRPM = ShootingRegressions.getRPM(distance);
        double currentRPM = shooter.getFlywheelVelocity() * 60.0;

        boolean atSpeed = Math.abs(currentRPM - targetRPM) <= tolerance;
        boolean timedOut = (Timer.getFPGATimestamp() - startTime) > kMaxWaitTime;

        if (atSpeed || timedOut) {
            return true;
        }
        return false;
    }
}