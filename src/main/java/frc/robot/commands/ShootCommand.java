package frc.robot.commands;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.shooting.ShootingRegressions;
import frc.robot.shooting.Targeting;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Shooter.ShooterControlMode;

/**
 * Adaptive shoot command using regression tables.
 *
 * <p>Spins up the flywheel to the RPM calculated from the current distance to
 * the HUB, waits for spin-up, then feeds balls through the indexer. Runs both
 * shooter and indexer in parallel — the shooter starts first, the indexer
 * joins after a delay. Cleans up both subsystems when the command ends.
 */
public class ShootCommand extends SequentialCommandGroup {

    /**
     * Creates a new ShootCommand.
     *
     * @param shooter    the shooter subsystem
     * @param indexer    the indexer subsystem
     * @param drivetrain the drivetrain (used to get current pose for distance)
     */
    public ShootCommand(Shooter shooter, Indexer indexer, Drivetrain drivetrain) {
        addCommands(
                parallel(
                        // Spin up flywheel based on distance regression
                        shooter.run(
                                () -> {
                                    var pose = drivetrain.getState().Pose;
                                    double distance = Targeting.getHorizontalDistance(pose);
                                    double motorRPS = ShootingRegressions.getMotorRPS(distance);
                                    shooter.setVelocity(motorRPS, ShooterControlMode.FF_AND_PID);
                                }),
                        // After spinup delay, feed balls
                        sequence(
                                waitSeconds(Constants.kShooterSpinupTime),
                                indexer
                                        .run(
                                                () ->
                                                        indexer.setDutyCycle(
                                                                Constants.kShooterIndexerDutyCycle))
                                        .withTimeout(Constants.kShooterFeedTime)))
                .finallyDo(
                        () -> {
                            // Reset both subsystems
                            shooter.setDutyCycle(0);
                            indexer.setDutyCycle(0);
                        }));
    }
}