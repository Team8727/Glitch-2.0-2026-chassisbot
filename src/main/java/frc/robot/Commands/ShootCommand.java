package frc.robot.Commands;

import Glitch.Lib.BaseMechanisms.Roller;
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
                        shooterRoller.setVelocity(motorSpeed / (1 - .268));// .1 = slip percentage
                      } else if (controlMode == ControlMode.FF_AND_PID) {
                        shooterRoller.setVelocity(motorSpeed / (1 - -.1), Roller.ControlMode.FF_AND_PID);
                      } else {
                        shooterRoller.m_loop.setNextR(VecBuilder.fill(0.95 * (Robot.firing.power) / (Math.PI * Robot.SHOOTER_FLYWHEEL_DIAMETER_METERS))); // In rad/sec
                        shooterRoller.m_loop.correct(VecBuilder.fill(shooterRoller.getVelocity()));
                        shooterRoller.m_loop.predict(0.020);
                        double nextVoltage = shooterRoller.m_loop.getU(0);
                        shooterRoller.setVoltage(nextVoltage);
                      }
                    }),
                    sequence(
                            waitSeconds(2),
                            indexer.run(() -> indexer.setDutyCycle(1))
                                    .withTimeout(1)
                    )
            ).finallyDo(() -> {
              shooterRoller.setDutyCycle(0);
              indexer.setDutyCycle(0);
              if (controlMode == ControlMode.STATE_SPACE) {
                shooterRoller.m_loop.setNextR(0.95 * (Robot.firing.power) / (Math.PI * Robot.SHOOTER_FLYWHEEL_DIAMETER_METERS));
                shooterRoller.m_loop.correct(VecBuilder.fill(shooterRoller.getVelocity()));
                shooterRoller.m_loop.predict(0.020);
                double nextVoltage = shooterRoller.m_loop.getU(0);
                shooterRoller.setVoltage(nextVoltage);
              }
            })
    );
  }

  public enum ControlMode {
    FEEDFORWARD,
    PID,
    FF_AND_PID,
    STATE_SPACE
  }
}
