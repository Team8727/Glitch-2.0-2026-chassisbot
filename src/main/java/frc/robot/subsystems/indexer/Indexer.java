package frc.robot.subsystems.indexer;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.NetworkTableLogger;
import frc.robot.Constants;

public class Indexer extends SubsystemBase {

    private final SparkMax leaderMotor;
    @SuppressWarnings("unused")
    private final SparkMax followerMotor;
    private final RelativeEncoder encoder;
    private final NetworkTableLogger logger = new NetworkTableLogger("Indexer");

    public Indexer() {
        // -- Leader config (CAN 4) --
        SparkMaxConfig leaderConfig = new SparkMaxConfig();
        leaderConfig
            .smartCurrentLimit(Constants.kIndexerLeaderCurrentLimit)
            .idleMode(IdleMode.kCoast)
            .inverted(false);
        leaderConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(0.07, 0, 0);

        leaderMotor = new SparkMax(Constants.kIndexerLeaderId, MotorType.kBrushless);
        leaderMotor.configure(leaderConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        encoder = leaderMotor.getEncoder();

        // -- Follower config (CAN 5, follow leader inverted) --
        SparkMaxConfig followerConfig = new SparkMaxConfig();
        followerConfig
            .smartCurrentLimit(Constants.kIndexerFollowerCurrentLimit)
            .idleMode(IdleMode.kCoast)
            .follow(Constants.kIndexerLeaderId, true);

        followerMotor = new SparkMax(Constants.kIndexerFollowerId, MotorType.kBrushless);
        followerMotor.configure(followerConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

        setDefaultCommand(run(() -> setDutyCycle(0)));
    }

    public void setDutyCycle(double dutyCycle) {
        leaderMotor.set(dutyCycle);
    }

    /** Returns the leader motor velocity in RPS. */
    public double getVelocity() {
        return encoder.getVelocity();
    }

    /** Returns the leader motor output current in amps. */
    public double getOutputCurrent() {
        return leaderMotor.getOutputCurrent();
    }

    @Override
    public void periodic() {
        // Publish indexer health for the dashboard every cycle.
        logger.logDouble("velocity", getVelocity());
        logger.logDouble("current", getOutputCurrent());
    }
}