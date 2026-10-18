package frc.robot.subsystems.shooter;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;

import static edu.wpi.first.units.Units.Volts;

public class Shooter extends SubsystemBase {

    public enum ShooterControlMode {
        PID,
        FEEDFORWARD,
        FF_AND_PID
    }

    // ---- Motors ----
    private final SparkMax leaderMotor;
    @SuppressWarnings("unused")
    private final SparkMax followerMotor1;
    @SuppressWarnings("unused")
    private final SparkMax followerMotor2;
    @SuppressWarnings("unused")
    private final SparkMax followerMotor3;
    private final SparkClosedLoopController closedLoopController;
    private final RelativeEncoder encoder;

    // ---- Feedforward ----
    private final SimpleMotorFeedforward feedforward =
        new SimpleMotorFeedforward(Constants.kShooterKS, Constants.kShooterKV, Constants.kShooterKA);

    // ---- SysId ----
    public SysIdRoutine sysIdRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(null, Volts.of(4), null,
            state -> { SignalLogger.writeString("FlywheelSysID_State", state.toString()); }),
        new SysIdRoutine.Mechanism(
            (voltage) -> setVoltage(voltage.in(Volts)), null, this));

    // ---- State-space control ----
    private final LinearSystem<N1, N1, N1> m_flywheelPlant =
        LinearSystemId.identifyVelocitySystem(Constants.kShooterKV, Constants.kShooterKA);

    private final KalmanFilter<N1, N1, N1> m_observer =
        new KalmanFilter<>(Nat.N1(), Nat.N1(), m_flywheelPlant,
            VecBuilder.fill(Constants.kShooterModelStdDev),
            VecBuilder.fill(Constants.kShooterEncoderStdDev), 0.020);

    private final LinearQuadraticRegulator<N1, N1, N1> m_controller =
        new LinearQuadraticRegulator<>(m_flywheelPlant,
            VecBuilder.fill(Constants.kShooterQelms),
            VecBuilder.fill(Constants.kShooterRelms), 0.020);

    public final LinearSystemLoop<N1, N1, N1> m_loop =
        new LinearSystemLoop<>(m_flywheelPlant, m_controller, m_observer, 12.0, 0.020);

    public Shooter() {
        // -- Leader config (CAN 6) --
        SparkMaxConfig leaderConfig = new SparkMaxConfig();
        leaderConfig
            .smartCurrentLimit(Constants.kShooterCurrentLimit)
            .idleMode(IdleMode.kCoast)
            .inverted(false);
        leaderConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(Constants.kShooterKP, 0, 0);
        leaderConfig.encoder
            .positionConversionFactor(Constants.kShooterEncoderConversion)
            .velocityConversionFactor(Constants.kShooterEncoderConversion);

        leaderMotor = new SparkMax(Constants.kShooterLeaderId, MotorType.kBrushless);
        leaderMotor.configure(leaderConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        closedLoopController = leaderMotor.getClosedLoopController();
        encoder = leaderMotor.getEncoder();

        // -- Follower 1 (CAN 7, follow leader non-inverted) --
        SparkMaxConfig f1Config = new SparkMaxConfig();
        f1Config
            .smartCurrentLimit(Constants.kShooterCurrentLimit)
            .idleMode(IdleMode.kCoast)
            .follow(Constants.kShooterLeaderId, false);
        followerMotor1 = new SparkMax(Constants.kShooterFollower1Id, MotorType.kBrushless);
        followerMotor1.configure(f1Config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

        // -- Follower 2 (CAN 8, follow leader inverted) --
        SparkMaxConfig f2Config = new SparkMaxConfig();
        f2Config
            .smartCurrentLimit(Constants.kShooterCurrentLimit)
            .idleMode(IdleMode.kCoast)
            .follow(Constants.kShooterLeaderId, true);
        followerMotor2 = new SparkMax(Constants.kShooterFollower2Id, MotorType.kBrushless);
        followerMotor2.configure(f2Config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

        // -- Follower 3 (CAN 9, follow leader inverted) --
        SparkMaxConfig f3Config = new SparkMaxConfig();
        f3Config
            .smartCurrentLimit(Constants.kShooterCurrentLimit)
            .idleMode(IdleMode.kCoast)
            .follow(Constants.kShooterLeaderId, true);
        followerMotor3 = new SparkMax(Constants.kShooterFollower3Id, MotorType.kBrushless);
        followerMotor3.configure(f3Config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

        setDefaultCommand(run(() -> setDutyCycle(0)));
    }

    /** Sets the duty cycle (percent output) on the leader motor. */
    public void setDutyCycle(double dutyCycle) {
        leaderMotor.set(dutyCycle);
    }

    /** Direct voltage control. */
    public void setVoltage(double volts) {
        leaderMotor.setVoltage(volts);
    }

    /**
     * Sets the flywheel velocity using pure feedforward (no PID feedback).
     *
     * @param speedRPS target velocity in RPS
     */
    public void setFFVoltageWithVelocity(double speedRPS) {
        setVoltage(feedforward.calculate(speedRPS));
    }

    /**
     * Sets the flywheel velocity using the specified control mode.
     *
     * @param speed target velocity in RPS
     * @param mode  PID, FEEDFORWARD, or FF_AND_PID
     */
    public void setVelocity(double speed, ShooterControlMode mode) {
        switch (mode) {
            case PID:
                closedLoopController.setSetpoint(speed, ControlType.kVelocity);
                break;
            case FEEDFORWARD:
                setVoltage(feedforward.calculate(speed));
                break;
            case FF_AND_PID:
                closedLoopController.setSetpoint(
                    speed, ControlType.kVelocity, ClosedLoopSlot.kSlot0,
                    feedforward.calculate(speed), ArbFFUnits.kVoltage);
                break;
        }
    }

    /** Convenience overload — uses PID mode. */
    public void setVelocity(double speed) {
        setVelocity(speed, ShooterControlMode.PID);
    }

    /**
     * Returns the current flywheel velocity in RPS, adjusted for gearing.
     */
    public double getFlywheelVelocity() {
        return getVelocity() / Constants.kFlywheelGearing;
    }

    /** Returns the raw encoder velocity in RPS. */
    public double getVelocity() {
        return encoder.getVelocity();
    }

    /** Returns the raw encoder position. */
    public double getPosition() {
        return encoder.getPosition();
    }

    public SimpleMotorFeedforward getFeedforward() {
        return feedforward;
    }

    /** Exposes the leader SparkMax for advanced use. */
    public SparkMax getLeaderMotor() {
        return leaderMotor;
    }

    /** Returns the leader motor output current in amps. */
    public double getOutputCurrent() {
        return leaderMotor.getOutputCurrent();
    }

    @Override
    public void periodic() {
        // State-space loop is driven externally (e.g. from Robot.teleopInit)
    }
}