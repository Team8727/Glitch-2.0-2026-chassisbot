package frc.robot.Commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.Subsystems.Indexer;
import frc.robot.Subsystems.ShooterRoller;
import frc.robot.Subsystems.Spindexer;

import static edu.wpi.first.wpilibj2.command.Commands.*;

public class ShootCommandFF extends SequentialCommandGroup {
  public ShootCommandFF(Indexer indexer, Spindexer spindexer, ShooterRoller shooterRoller, double fixedvalue) {
    addCommands(
            parallel(
                    shooterRoller.run(() -> {
                      double speed;
                        if (fixedvalue == 0){
                          speed = 0.95 * (Robot.firing.power) / (Math.PI * Robot.SHOOTER_FLYWHEEL_RADIUS_METERS);
                        }
                        else
                        {
                          speed = 0.95 * fixedvalue/(Math.PI * Robot.SHOOTER_FLYWHEEL_RADIUS_METERS);
                        }
                      
                    
                      shooterRoller.setFFVoltageWithVelocity(speed);
                    }),
                    sequence(
                            waitSeconds(1),
                            parallel(
                                    indexer.run(() -> indexer.setSpeedDutyCycle(1)),
                                    spindexer.run(() -> spindexer.setSpeedDutyCycle(.5))
                            ).withTimeout(1)
                    )
            )//.withTimeout(2.5)
    );

  }
}
