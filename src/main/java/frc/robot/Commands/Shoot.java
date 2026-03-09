package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.Subsystems.Indexer;
import frc.robot.Subsystems.ShooterRoller;
import frc.robot.Subsystems.Spindexer;
import frc.robot.controller.CTReSwerveControls;

import static edu.wpi.first.wpilibj2.command.Commands.parallel;
import static edu.wpi.first.wpilibj2.command.Commands.run;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;
import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;

public class Shoot extends SequentialCommandGroup {
  public Shoot(Indexer indexer, Spindexer spindexer, ShooterRoller shooterRoller) {
    addCommands(
        parallel(
            run(() -> shooterRoller.setSpeedVelocity(Robot.firing.power*6.6), shooterRoller),
            sequence(
                waitSeconds(1.5),
                parallel(
                  run(() -> indexer.setSpeedDutyCycle(1), indexer),
                  run(() -> spindexer.setSpeedDutyCycle(.5), spindexer)
                )
            )
        )
    );
  }
}
