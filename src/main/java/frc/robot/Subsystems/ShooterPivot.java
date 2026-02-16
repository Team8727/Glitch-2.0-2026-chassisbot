
package frc.robot.Subsystems;

import Glitch.Lib.BaseMechanisms.Pivot;
import Glitch.Lib.Motors.SparkMaxMotor;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;

public class ShooterPivot extends Pivot {

  private static final int CANID = 57;
  private static final SparkMaxConfig config = new SparkMaxConfig();
  static {
    config
      .smartCurrentLimit(60)
      .idleMode(SparkMaxConfig.IdleMode.kBrake)
      .closedLoop
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder);}

  public ShooterPivot() {
    super(
      new SparkMaxMotor(
        config,
        CANID,
        FeedbackSensor.kAbsoluteEncoder),
      0,//TODO: find values
      0,
      0,
      1);
  }

  /** This method will be called once per scheduler run */
  @Override
  public void periodic() {
    super.periodic();
    // Add any additional periodic logic here
  }
}
