
package frc.robot.Subsystems;

import Glitch.Lib.BaseMechanisms.Pivot;
import Glitch.Lib.Motors.SparkMaxMotor;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;

public class IntakePivot extends Pivot {
  public enum IntakePosition {
    UP(0),//TODO: find values
    MID(100),
    DOWN(124);

    private final double degrees;
    private IntakePosition(double degrees) { this.degrees = degrees; }

    public double getDegrees() {
      return degrees;
    }
  }

  private static final int CANID = 4;
  private static final SparkMaxConfig config = new SparkMaxConfig();
  static {
    config
      .smartCurrentLimit(40)
      .idleMode(SparkMaxConfig.IdleMode.kCoast)
      .inverted(false)
      .closedLoop
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        .pid(10,0,0);
  }

  public IntakePivot() {
    super(
      new SparkMaxMotor(
        config,
        CANID,
        FeedbackSensor.kAbsoluteEncoder),
      0,//TODO: find values
      10000,
      1000,
      1);
    setDisabled(true);
    setDutyCycle(.5);
  }

  /** This method will be called once per scheduler run */
  @Override
  public void periodic() {
    super.periodic();
    if (getPosition()*360 > 60) {
      setDutyCycle(0);
    } else {
      setDutyCycle(.5);
    }
    // Add any additional periodic logic here
  }
}
