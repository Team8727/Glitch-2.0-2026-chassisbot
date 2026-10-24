package frc.robot.subsystems.intake;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.NetworkTableLogger;
import frc.robot.Constants;

public class Intake extends SubsystemBase {

    private final SparkMax motor;
    private final RelativeEncoder encoder;
    private final NetworkTableLogger logger = new NetworkTableLogger("Intake");
    private boolean m_intaking = false;

    public Intake() {
        SparkMaxConfig config = new SparkMaxConfig();
        config
            .smartCurrentLimit(Constants.kIntakeCurrentLimit)
            .idleMode(IdleMode.kCoast)
            .inverted(false);
        config.closedLoop
            .pid(0.07, 0, 0);

        motor = new SparkMax(Constants.kIntakeId, MotorType.kBrushless);
        motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        encoder = motor.getEncoder();

        setDefaultCommand(run(() -> setDutyCycle(0)));
    }

    public void setDutyCycle(double dutyCycle) {
        motor.set(dutyCycle);
    }

    public boolean isIntaking() {
        return m_intaking;
    }

    public void setIntaking(boolean v) {
        m_intaking = v;
    }

    /** Returns the intake motor velocity in RPS. */
    public double getVelocity() {
        return encoder.getVelocity();
    }

    /** Returns the intake motor output current in amps. */
    public double getOutputCurrent() {
        return motor.getOutputCurrent();
    }

    @Override
    public void periodic() {
        // Publish intake health for the dashboard every cycle.
        logger.logDouble("velocity", getVelocity());
        logger.logDouble("current", getOutputCurrent());
    }
}