package Glitch.Lib.Motors;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.*;
import com.revrobotics.spark.config.SparkMaxConfig;

import java.util.Set;

import static Glitch.Lib.Motors.SparkConfigurator.getSparkMax;

public class SparkMaxMotor implements Motor{
  private final SparkMax motor;
  private final SparkClosedLoopController motorController;
  private final boolean hasAbsoluteEncoder;

  public SparkMaxMotor(SparkMaxConfig config, int CANID, FeedbackSensor encoderType) {
    motor = getSparkMax(
      CANID,
      SparkLowLevel.MotorType.kBrushless,
      false,
      Set.of(),
      Set.of(
        SparkConfigurator.LogData.POSITION,
        SparkConfigurator.LogData.VELOCITY,
        SparkConfigurator.LogData.VOLTAGE,
        SparkConfigurator.LogData.CURRENT));

    motor.configure(
      config,
      ResetMode.kNoResetSafeParameters,
      PersistMode.kNoPersistParameters);

    motorController = motor.getClosedLoopController();

    hasAbsoluteEncoder = encoderType == FeedbackSensor.kAbsoluteEncoder;
  }

  @Override
  public void setVelocity(double velocity) {
    motorController.setSetpoint(velocity, SparkBase.ControlType.kVelocity);
  }

  @Override
  public void setDutyCycle(double dutyCycle) {
    motorController.setSetpoint(dutyCycle, SparkBase.ControlType.kDutyCycle);
  }

  @Override
  public void setPosition(double position, double feedforward) {
    motorController.setSetpoint(
      position,
      SparkBase.ControlType.kPosition,
      ClosedLoopSlot.kSlot0,
      feedforward);
  }

  @Override
  public void setPosition(double position) {
    motorController.setSetpoint(
      position,
      SparkBase.ControlType.kPosition);
  }

  @Override
  public double getPosition() {
    if (hasAbsoluteEncoder) {
      return motor.getAbsoluteEncoder().getPosition();
    } else {
      return motor.getEncoder().getPosition();
    }
  }

  @Override
  public double getCurrent() {
    return motor.getOutputCurrent();
  }

  @Override
  public double getVelocity() {
    if (hasAbsoluteEncoder) {
      return motor.getAbsoluteEncoder().getVelocity();
    } else {
      return motor.getEncoder().getVelocity();
    }
  }

  @Override
  public boolean getForwardLimitSwitch() {
    return motor.getForwardLimitSwitch().isPressed();
  }

  @Override
  public boolean getReverseLimitSwitch() {
    return motor.getReverseLimitSwitch().isPressed();
  }
}