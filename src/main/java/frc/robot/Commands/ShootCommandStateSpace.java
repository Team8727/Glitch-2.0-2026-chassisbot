package frc.robot.Commands;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.Subsystems.Indexer;
import frc.robot.Subsystems.ShooterRoller;
import frc.robot.Subsystems.Spindexer;

import static edu.wpi.first.wpilibj2.command.Commands.*;

public class ShootCommandStateSpace extends SequentialCommandGroup {
  public ShootCommandStateSpace(Indexer indexer, Spindexer spindexer, ShooterRoller shooterRoller) {
    addCommands(
            parallel(
                    shooterRoller.run(() -> {
                      shooterRoller.m_loop.setNextR(VecBuilder.fill(0.95 * (Robot.firing.power) / (Math.PI * Robot.SHOOTER_FLYWHEEL_RADIUS_METERS))); // In rad/sec
                      shooterRoller.m_loop.correct(VecBuilder.fill(shooterRoller.getVelocity()));
                      shooterRoller.m_loop.predict(0.020);
                      double nextVoltage = shooterRoller.m_loop.getU(0);
                      shooterRoller.setSpeedVoltage(nextVoltage);
                    }),
                    sequence(
                            waitSeconds(1.5),
                            parallel(
                                    indexer.run(() -> indexer.setSpeedDutyCycle(1)),
                                    spindexer.run(() -> spindexer.setSpeedDutyCycle(.5))
                            ).withTimeout(1)
                    )
            ).finallyDo(() -> {
              shooterRoller.m_loop.setNextR(0.95 * (Robot.firing.power) / (Math.PI * Robot.SHOOTER_FLYWHEEL_RADIUS_METERS));
              shooterRoller.m_loop.correct(VecBuilder.fill(shooterRoller.getVelocity()));
              shooterRoller.m_loop.predict(0.020);
              double nextVoltage = shooterRoller.m_loop.getU(0);
              shooterRoller.setSpeedVoltage(nextVoltage);
            })//.withTimeout(2.5)
    );
  }
}
