package frc.robot.Subsystems;

import Glitch.Lib.BaseMechanisms.Roller;
import Glitch.Lib.Motors.SparkMaxMotor;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;


public class Spindexer extends Roller {
  private static final int CANID = 16;
  private static final SparkMaxConfig config = new SparkMaxConfig();
  static {
    config
      .smartCurrentLimit(60)
      .idleMode(SparkMaxConfig.IdleMode.kBrake)
      .inverted(false)
      .closedLoop
        .pid(0, 0, 0);//TODO: find these values
  }

  public Spindexer() {
    super(new SparkMaxMotor(config, CANID, FeedbackSensor.kPrimaryEncoder));
  }
    /** This method will be called once per scheduler run */
  @Override
  public void periodic() {
    super.periodic();
    // Add any additional periodic logic here
  }
}
