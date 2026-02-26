
package frc.robot.Subsystems;

import Glitch.Lib.BaseMechanisms.Pivot;
import Glitch.Lib.Motors.SparkMaxMotor;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;

public class ShooterPivot extends Pivot {
  public enum MaxShooterAngles {
    UP(20),//TODO: find values
    DOWN(0);

    private final double degrees;
    private MaxShooterAngles(double degrees) { this.degrees = degrees; }

    public double getDegrees() {
      return degrees;
    }
  }

  private static final int CANID = 9;
  private static final SparkMaxConfig config = new SparkMaxConfig();
  static {
    config
      .smartCurrentLimit(40)
      .idleMode(SparkMaxConfig.IdleMode.kBrake)
      .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder);}

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
