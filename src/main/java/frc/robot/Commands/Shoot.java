package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Subsystems.Indexer;
import frc.robot.Subsystems.ShooterRollers;
import frc.robot.Subsystems.Spindexer;

import static edu.wpi.first.wpilibj2.command.Commands.parallel;
import static edu.wpi.first.wpilibj2.command.Commands.run;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;
import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;

public class Shoot extends SequentialCommandGroup {

  public Shoot(Indexer indexer, Spindexer spindexer, ShooterRollers shooterRollers) {
    addCommands(
        parallel(
            run(() -> shooterRollers.setSpeedDutyCycle(.5), shooterRollers),
            sequence(
                waitSeconds(0.25),
                run(() -> indexer.setSpeedDutyCycle(1), indexer)
            ),
            sequence(
                waitSeconds(0.3),
                run(() -> spindexer.setSpeedDutyCycle(.5), spindexer)
            )
        )
    );
  }
}
