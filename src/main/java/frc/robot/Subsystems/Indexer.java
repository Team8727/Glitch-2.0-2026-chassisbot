package frc.robot.Subsystems;

import Glitch.Lib.BaseMechanisms.Roller;
import Glitch.Lib.Motors.SparkMaxMotor;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;

public class Indexer extends Roller {
    private static final int M1CANID = 4;
    private static final SparkMaxConfig M1config = new SparkMaxConfig();
    private static final int M2CANID = 5;
    private static final SparkMaxConfig M2config = new SparkMaxConfig();
    static {
        M1config
                .smartCurrentLimit(60)
                .idleMode(SparkMaxConfig.IdleMode.kCoast)
                .inverted(true)
                .disableFollowerMode()
                .closedLoop
                .pid(0, 0, 0); //TODO: Tune PID values
    }
    static {
        M2config
                .smartCurrentLimit(60)
                .idleMode(SparkMaxConfig.IdleMode.kCoast)
                .follow(M1CANID, true)
                .closedLoop
                .pid(0, 0, 0); //TODO: Tune PID values
    }

    // Keep a reference so follower stays constructed/configured
    private final SparkMaxMotor followerMotor;

    public Indexer() {
        super(new SparkMaxMotor(M1config, M1CANID, FeedbackSensor.kPrimaryEncoder));
        followerMotor = new SparkMaxMotor(M2config, M2CANID, FeedbackSensor.kPrimaryEncoder);
        setDefaultCommand(run(() -> setSpeedDutyCycle(0)));
    }

    /** This method will be called once per scheduler run */
    @Override
    public void periodic() {
        super.periodic();
        // Add any additional periodic logic here
    }
}
