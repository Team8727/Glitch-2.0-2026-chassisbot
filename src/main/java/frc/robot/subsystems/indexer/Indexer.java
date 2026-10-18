package frc.robot.subsystems.indexer;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Indexer extends SubsystemBase {

    private final SparkMax leaderMotor;
    @SuppressWarnings("unused")
    private final SparkMax followerMotor;

    public Indexer() {
        // -- Leader config (CAN 4) --
        SparkMaxConfig leaderConfig = new SparkMaxConfig();
        leaderConfig
            .smartCurrentLimit(Constants.kIndexerLeaderCurrentLimit)
            .idleMode(IdleMode.kCoast)
            .inverted(false);
        leaderConfig.closedLoop
            .pid(0.07, 0, 0);

        leaderMotor = new SparkMax(Constants.kIndexerLeaderId, MotorType.kBrushless);
        leaderMotor.configure(leaderConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

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

    @Override
    public void periodic() {
        // No-op for now
    }
}