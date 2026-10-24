package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.shooter.Shooter;

/**
 * One-button shoot command.
 *
 * <p>ZR just shoots — no heading control, no auto-aim. The flywheel spins up
 * immediately and the indexer feeds once at speed. Aim the robot yourself (or
 * tap A to face the HUB) before firing.
 *
 * <p>Power/RPM is computed from the projectile solver for the alliance HUB, so
 * whatever heading you fire at, the note is launched at the correct speed for
 * that distance to the HUB.
 */
public class SmartShootCommand extends SequentialCommandGroup {

    public SmartShootCommand(Shooter shooter, Indexer indexer, Drivetrain drivetrain) {
        addCommands(
                new ShootCommand(shooter, indexer, drivetrain)
                        .finallyDo(() -> {
                            shooter.setDutyCycle(0);
                            indexer.setDutyCycle(0);
                        }));
    }
}
