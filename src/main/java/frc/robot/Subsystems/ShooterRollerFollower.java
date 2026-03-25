package frc.robot.Subsystems;

import Glitch.Lib.BaseMechanisms.Roller;
import Glitch.Lib.Motors.SparkMaxMotor;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;

public class ShooterRollerFollower extends Roller {
    private static final int CANID = 8;
    private static final SparkMaxConfig config = new SparkMaxConfig();
    static {
        config
                .smartCurrentLimit(60)
                .idleMode(SparkMaxConfig.IdleMode.kCoast)
                .inverted(false)
                .follow(7)
                .closedLoop
                .pid(.07, 0, 0); //TODO: Tune PID values
        config
                .encoder
                .positionConversionFactor(0.017453299835324287)
                .velocityConversionFactor(0.017453299835324287);
    }

    public ShooterRollerFollower() {
        super(new SparkMaxMotor(config, CANID, FeedbackSensor.kPrimaryEncoder));
        setDefaultCommand(run(() -> setSpeedDutyCycle(0)));
    }

    /** This method will be called once per scheduler run */
    @Override
    public void periodic() {
        super.periodic();
        // Add any additional periodic logic here
    }
}
