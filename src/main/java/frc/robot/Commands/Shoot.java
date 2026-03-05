package frc.robot.Commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Drivetrain.CTRESwerveDrivetrain;
import frc.robot.Robot;
import frc.robot.Subsystems.Indexer;
import frc.robot.Subsystems.ShooterRollers;
import frc.robot.Subsystems.Spindexer;
import frc.robot.controller.CTReSwerveControls;

import static edu.wpi.first.wpilibj2.command.Commands.parallel;
import static edu.wpi.first.wpilibj2.command.Commands.run;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;
import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;
import static frc.robot.controller.CTReSwerveControls.MaxSpeed;

public class Shoot extends SequentialCommandGroup {
  private CTReSwerveControls swerveController;

  public Shoot(Indexer indexer, Spindexer spindexer, ShooterRollers shooterRollers) {
    addCommands(
        parallel(
            run(() -> shooterRollers.setSpeedVelocityM1(Robot.firing.power*4.02985+10), shooterRollers),
            sequence(
                waitSeconds(0.5),
                parallel(
                  run(() -> indexer.setSpeedDutyCycle(1), indexer),
                  run(() -> spindexer.setSpeedDutyCycle(.5), spindexer)
                )
            )
        )
    );
  }
}
