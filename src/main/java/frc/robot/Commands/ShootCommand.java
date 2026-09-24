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
  // Slip compensation percentage for FF+PID control mode (15%)
  private static final double SLIP_PERCENTAGE_FF_PID = 0.15;
  // Speed factor for STATE_SPACE control mode
  private static final double STATE_SPACE_SPEED_FACTOR = 0.95;
  // Fallback muzzle velocity (m/s) used when the projectile solver has no valid solution.
  // Instead of a constant, we estimate from horizontal distance so the power still varies
  // with how far the robot is from the hub — copied into each lambda below.
  private static final double GRAVITY_MPS2 = 9.81;

  /**
   * Estimate the required muzzle velocity (m/s) for the fixed shooter angle at a given horizontal
   * distance.  Uses the standard projectile-motion equation:
   * <pre>
   *   v² = g·d² / (2·cos²θ·(d·tanθ − Δz))
   * </pre>
   * Falls back to a reasonable minimum if the target is too close.
   */
  private static double estimateMuzzleVelocity(double horizontalDistanceMeters) {
    double dz = Robot.HUB_Z_METERS - Robot.SHOOTER_HEIGHT_METERS;
    double theta = Math.toRadians(Robot.SHOOTER_ANGLE_DEGREES);
    double cosTheta = Math.cos(theta);
    double tanTheta = Math.tan(theta);

    double denom = horizontalDistanceMeters * tanTheta - dz;
    if (denom <= 0.001) return 8.0;               // target too close for the line-of-sight
    double vSq = GRAVITY_MPS2 * horizontalDistanceMeters * horizontalDistanceMeters
               / (2.0 * cosTheta * cosTheta * denom);
    return Math.sqrt(Math.max(vSq, 36.0));        // floor = 6 m/s (never return 0)
  }

  public ShootCommand(Indexer indexer, ShooterRoller shooterRoller, double fixedLinearVelocity, ControlMode controlMode) {
    addCommands(
            parallel(
                    shooterRoller.run(() -> {
                      double speedCoefficient = 1 / (Math.PI * Robot.SHOOTER_FLYWHEEL_DIAMETER_METERS); // Used to convert to rps with multiplier
                      double muzzleVelocity = (fixedLinearVelocity == 0)
                              ? (Robot.firing.isValid ? Robot.firing.power : estimateMuzzleVelocity(Robot.firing.horizontalDistance))
                              : fixedLinearVelocity; // Manual shoot override
                      double flywheelSpeed = speedCoefficient * muzzleVelocity;
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
                double muzzleVelocity = (fixedLinearVelocity == 0)
                        ? (Robot.firing.isValid ? Robot.firing.power : estimateMuzzleVelocity(Robot.firing.horizontalDistance))
                        : fixedLinearVelocity; // Manual shoot override
                double flywheelSpeed = speedCoefficient * muzzleVelocity;
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
