package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.Subsystems.Indexer;
import frc.robot.Subsystems.ShooterRoller;
import frc.robot.Subsystems.Spindexer;

import static edu.wpi.first.wpilibj2.command.Commands.*;

public class Shoot extends SequentialCommandGroup {
  public Shoot(Indexer indexer, Spindexer spindexer, ShooterRoller shooterRoller) {
    addCommands(
            parallel(
                    shooterRoller.run(() -> shooterRoller.setSpeedVelocity(Robot.firing.power * Math.PI * Robot.SHOOTER_LOSS_COMPENSATION)),
                    sequence(
                            waitSeconds(1.5),
                            parallel(
                                    indexer.run(() -> indexer.setSpeedDutyCycle(1)),
                                    spindexer.run(() -> spindexer.setSpeedDutyCycle(.5))
                            )
                    )
            )
    );
  }
}
