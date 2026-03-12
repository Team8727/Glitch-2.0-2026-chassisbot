package frc.robot.Subsystems;

import Glitch.Lib.BaseMechanisms.Roller;
import Glitch.Lib.Motors.SparkMaxMotor;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;


public class Spindexer extends Roller {
  private static final int CANID = 5;
  private static final SparkMaxConfig config = new SparkMaxConfig();
  static {
    config
      .smartCurrentLimit(40)
      .idleMode(SparkMaxConfig.IdleMode.kCoast)
      .inverted(false)
      .closedLoop
        .pid(1, 0, 0)
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder);//TODO: find these values
  }

  public Spindexer() {
    super(new SparkMaxMotor(config, CANID, FeedbackSensor.kPrimaryEncoder));
    setDefaultCommand(run(() -> setSpeedDutyCycle(0)));
//    setSpeedVelocity(1);
  }
    /** This method will be called once per scheduler run */
  @Override
  public void periodic() {
    super.periodic();
    // Add any additional periodic logic here
  }
}
