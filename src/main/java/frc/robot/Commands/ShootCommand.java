package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.Subsystems.Indexer;
import frc.robot.Subsystems.LEDSubsystem;
import frc.robot.Subsystems.ShooterRoller;

import static edu.wpi.first.wpilibj2.command.Commands.*;

public class ShootCommand extends SequentialCommandGroup {
  public ShootCommand(Indexer indexer, ShooterRoller shooterRoller, LEDSubsystem leds) {
    addCommands(
            parallel(
                    shooterRoller.run(() -> {
                      double speed;
                      if (Robot.SHOOT_POWER_OVERRIDE) {
                        speed = 6.6 * Math.PI * Robot.SHOOTER_LOSS_COMPENSATION; // Old method
                      } else {
                        //speed = Robot.firing.power * Math.PI * Robot.SHOOTER_LOSS_COMPENSATION; // Old Method
                        speed = 0.95 * (Robot.firing.power) / (Math.PI * Robot.SHOOTER_FLYWHEEL_RADIUS_METERS);
                      }
                      shooterRoller.setSpeedVelocity(speed);
                      leds.shootPatterns(shooterRoller.getFlywheelVelocity(), speed);
                    }),
                    sequence(
                            waitSeconds(2),
                            indexer.run(() -> indexer.setSpeedDutyCycle(-.9))
                                    .withTimeout(1)
                    )
            ).finallyDo(() -> {leds.endCommand();})//.withTimeout(2.5)
    );
  }
}
