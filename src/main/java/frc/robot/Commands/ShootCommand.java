package frc.robot.Commands;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.Subsystems.Indexer;
import frc.robot.Subsystems.ShooterRoller;

import static edu.wpi.first.wpilibj2.command.Commands.*;

public class ShootCommand extends SequentialCommandGroup {
  public ShootCommand(Indexer indexer, ShooterRoller shooterRoller, double fixedLinearVelocity, ControlMode controlMode) {
    addCommands(
            parallel(
                    shooterRoller.run(() -> {
                      double speedCoefficient = 1 / (Math.PI * Robot.SHOOTER_FLYWHEEL_DIAMETER_METERS); // Used to convert to rps with multiplier
                      double flywheelSpeed = speedCoefficient * ((fixedLinearVelocity == 0) ? Robot.firing.power : fixedLinearVelocity); // Whether to use manual shoot override or not
                      double motorSpeed = flywheelSpeed * (24.0 /15);
                      if (controlMode == ControlMode.FEEDFORWARD) {
                        shooterRoller.setFFVoltageWithVelocity(1 * flywheelSpeed);
                      } else if (controlMode == ControlMode.PID) {
                        shooterRoller.setSpeedVelocity(motorSpeed/(1-.268));// .1 = slip percentage
                      } else {
                        shooterRoller.m_loop.setNextR(VecBuilder.fill(0.95 * (Robot.firing.power) / (Math.PI * Robot.SHOOTER_FLYWHEEL_DIAMETER_METERS))); // In rad/sec
                        shooterRoller.m_loop.correct(VecBuilder.fill(shooterRoller.getVelocity()));
                        shooterRoller.m_loop.predict(0.020);
                        double nextVoltage = shooterRoller.m_loop.getU(0);
                        shooterRoller.setSpeedVoltage(nextVoltage);
                      }
                    }),
                    sequence(
                            waitSeconds(2),
                            indexer.run(() -> indexer.setSpeedDutyCycle(1))
                                    .withTimeout(1)
                    )
            ).finallyDo(() -> {
              if (controlMode == ControlMode.STATE_SPACE) {
                shooterRoller.m_loop.setNextR(0.95 * (Robot.firing.power) / (Math.PI * Robot.SHOOTER_FLYWHEEL_DIAMETER_METERS));
                shooterRoller.m_loop.correct(VecBuilder.fill(shooterRoller.getVelocity()));
                shooterRoller.m_loop.predict(0.020);
                double nextVoltage = shooterRoller.m_loop.getU(0);
                shooterRoller.setSpeedVoltage(nextVoltage);
              }
            })
    );
  }

  public enum ControlMode {
    FEEDFORWARD,
    PID,
    STATE_SPACE
  }
}
