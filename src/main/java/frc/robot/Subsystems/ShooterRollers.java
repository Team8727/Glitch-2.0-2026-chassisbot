package frc.robot.Subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterRollers extends SubsystemBase {
  private final ShooterRoller shooterRoller = new ShooterRoller();
  private final ShooterRoller2 shooterRoller2  = new ShooterRoller2();

  public void setSpeedDutyCycle(double speedDutyCycle) {
    shooterRoller.setSpeedDutyCycle(speedDutyCycle);
    shooterRoller2.setSpeedDutyCycle(speedDutyCycle);
  }

  public void setSpeedVelocity(double velocity) {
    shooterRoller.setSpeedVelocity(velocity);
    shooterRoller2.setSpeedVelocity(velocity);
  }

  public void setSpeedVelocityM1(double velocity) {
    shooterRoller.setSpeedVelocity(velocity);
  }

  public void setSpeedVelocityM2(double velocity) {
    shooterRoller2.setSpeedVelocity(velocity);
  }

  public void setSpeedDutyCycleM1(double dutyCycle) {
    shooterRoller.setSpeedDutyCycle(dutyCycle);
  }

}
