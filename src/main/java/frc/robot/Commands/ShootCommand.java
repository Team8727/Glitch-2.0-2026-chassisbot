package frc.robot.Commands;

import Glitch.Lib.BaseMechanisms.Roller;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.Subsystems.Indexer;
import frc.robot.Subsystems.ShooterRoller;

import static edu.wpi.first.wpilibj2.command.Commands.*;

public class ShootCommand extends SequentialCommandGroup {
  // Gear ratio between the flywheel and the motor (output/input)
  private static final double FLYWHEEL_GEAR_RATIO = 24.0 / 15.0;
  // Slip compensation percentage for PID control mode (26.8%)
  private static final double SLIP_PERCENTAGE_PID = 0.268;
  // Slip compensation percentage for FF+PID control mode (10%)
  private static final double SLIP_PERCENTAGE_FF_PID = 0.1;
  // Speed factor for STATE_SPACE control mode
  private static final double STATE_SPACE_SPEED_FACTOR = 0.95;

  public ShootCommand(Indexer indexer, ShooterRoller shooterRoller, double fixedLinearVelocity, ControlMode controlMode) {
    addCommands(
            parallel(
                    shooterRoller.run(() -> {
                      double speedCoefficient = 1 / (Math.PI * Robot.SHOOTER_FLYWHEEL_DIAMETER_METERS); // Used to convert to rps with multiplier
                      double flywheelSpeed = speedCoefficient * ((fixedLinearVelocity == 0) ? Robot.firing.power : fixedLinearVelocity); // Whether to use manual shoot override or not
                      double motorSpeed = flywheelSpeed * FLYWHEEL_GEAR_RATIO;
                      if (controlMode == ControlMode.FEEDFORWARD) {
                        shooterRoller.setFFVoltageWithVelocity(1 * flywheelSpeed);
                      } else if (controlMode == ControlMode.PID) {
                        shooterRoller.setVelocity(motorSpeed / (1 - SLIP_PERCENTAGE_PID));
                      } else if (controlMode == ControlMode.FF_AND_PID) {
                        shooterRoller.setVelocity(motorSpeed / (1 - SLIP_PERCENTAGE_FF_PID), Roller.ControlMode.FF_AND_PID);
                      } else {
                        shooterRoller.m_loop.setNextR(VecBuilder.fill(STATE_SPACE_SPEED_FACTOR * flywheelSpeed));
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
              if (controlMode == ControlMode.STATE_SPACE) {
                double speedCoefficient = 1 / (Math.PI * Robot.SHOOTER_FLYWHEEL_DIAMETER_METERS);
                double flywheelSpeed = speedCoefficient * ((fixedLinearVelocity == 0) ? Robot.firing.power : fixedLinearVelocity);
                double motorSpeed = flywheelSpeed * FLYWHEEL_GEAR_RATIO;
                shooterRoller.m_loop.setNextR(VecBuilder.fill(STATE_SPACE_SPEED_FACTOR * flywheelSpeed));
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
