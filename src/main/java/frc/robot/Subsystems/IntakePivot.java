
package frc.robot.Subsystems;

import Glitch.Lib.BaseMechanisms.Pivot;
import Glitch.Lib.Motors.SparkMaxMotor;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class IntakePivot extends Pivot {
  public enum IntakePosition {
    UP(20),//TODO: find values
    DOWN(30);

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
      .idleMode(SparkMaxConfig.IdleMode.kBrake)
      .closedLoop
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder);}

  public IntakePivot() {
    super(
      new SparkMaxMotor(
        config,
        CANID,
        FeedbackSensor.kAbsoluteEncoder),
      0,//TODO: find values
      0,
      0,
      1);
    setDefaultCommand(run(() -> setPosition(IntakePosition.DOWN.getDegrees())));
  }

  /** This method will be called once per scheduler run */
  @Override
  public void periodic() {
    super.periodic();
    // Add any additional periodic logic here
  }
}
