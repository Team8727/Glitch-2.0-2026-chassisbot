package frc.robot.Subsystems;

import Glitch.Lib.BaseMechanisms.Roller;
import Glitch.Lib.Motors.SparkMaxMotor;
import com.ctre.phoenix6.SignalLogger;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.LinearSystemId;

import static edu.wpi.first.units.Units.Volts;

public class ShooterRoller extends Roller {
    private static final int CANID = 7;
    private static final SparkMaxConfig config = new SparkMaxConfig();
    static {
        config
                .smartCurrentLimit(60)
                .idleMode(SparkMaxConfig.IdleMode.kCoast)
                .inverted(false)
                .closedLoop
                .pid(.07, 0, 0); // TODO: Tune PID values
//                .feedForward // Doesn't work, is a known REV issue, use SimpleMotorFeedforward *or* FF incorporated into state space
//                .sva(0.351, 0.127, 0.052, ClosedLoopSlot.kSlot0); // Found using sysID
        config
                .encoder
                .positionConversionFactor(0.017453299835324287) // To get output in RPS
                .velocityConversionFactor(.017453299835324287);
                // 0.017453299835324287 was the conversion factor previously. This value is close to 1/60, so I think using this outputs RPS.
    }

    private double kS = 0.35091; // Found using SysID
    private double kV = 0.12701;
    private double kA = 0.052063;
    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(kS, kV, kA);

    public SysIdRoutine sysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                    null,
                    Volts.of(4),
                    null,
                    state -> {
                        this.logger.logString("FlywheelSysID_State", state.toString());
                        SignalLogger.writeString("FlywheelSysID_State_SignalLogger", state.toString());
                    }
            ),
            new SysIdRoutine.Mechanism(
                    (voltage) -> setSpeedVoltage(voltage.in(Volts)),
                    null, // No log consumer, since data is recorded by URCL
                    this
            )
    );

// -=-=-=-=-=- State Space Flywheel Control logic below + see Trigger in Driver1DefaultBindings + Robot teleopInit() (using a model to determine speed of flywheel) -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

    public static final double kSpinupRadPerSec = 53.0; //Was 500 originally, change to something reasonable for max speed. Will not be a variable, it will be a value from projectile solver multiplied by 2pi + small mech. constant.

    private static final double kFlywheelMomentOfInertia = 0.00193; // kg * m^2
    // Reduction between motors and encoder, as output over input. If the flywheel spins slower than
    // the motors, this number should be greater than one.
    private static final double kFlywheelGearing = 0.5;

    // The plant holds a state-space model of our flywheel. This system has the following properties:
    //
    // States: [velocity], in radians per second.
    // Inputs (what we can "put in"): [voltage], in volts.
    // Outputs (what we can measure): [velocity], in radians per second.
//    private final LinearSystem<N1, N1, N1> m_flywheelPlant =
//            LinearSystemId.createFlywheelSystem(
//                    DCMotor.getNEO(2), kFlywheelMomentOfInertia, kFlywheelGearing);

    // TODO: Try using this instead of the above to add FF into this system.
    private final LinearSystem<N1, N1, N1> m_flywheelPlant =
            LinearSystemId.identifyVelocitySystem(kV, kA);

    // The observer fuses our encoder data and voltage inputs to reject noise.
    private final KalmanFilter<N1, N1, N1> m_observer =
            new KalmanFilter<>(
                    Nat.N1(),
                    Nat.N1(),
                    m_flywheelPlant,
                    VecBuilder.fill(3.0), // How accurate we think our model is
                    VecBuilder.fill(0.01), // How accurate we think our encoder
                    // data is
                    0.020);

    // A LQR uses feedback to create voltage commands.
    private final LinearQuadraticRegulator<N1, N1, N1> m_controller =
            new LinearQuadraticRegulator<>(
                    m_flywheelPlant,
                    VecBuilder.fill(8.0), // qelms. Velocity error tolerance, in radians per second. Decrease
                    // this to more heavily penalize state excursion, or make the controller behave more
                    // aggressively.
                    VecBuilder.fill(8.0), // relms. Control effort (voltage) tolerance. Decrease this to more
                    // heavily penalize control effort, or make the controller less aggressive. 12 is a good
                    // starting point because that is the (approximate) maximum voltage of a battery.
                    0.020); // Nominal time between loops. 0.020 for TimedRobot, but can be
    // lower if using notifiers.

    // The state-space loop combines a controller, observer, feedforward and plant for easy control.
    public final LinearSystemLoop<N1, N1, N1> m_loop =
            new LinearSystemLoop<>(m_flywheelPlant, m_controller, m_observer, 12.0, 0.020);

// -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

    public ShooterRoller() {
        super(new SparkMaxMotor(config, CANID, FeedbackSensor.kPrimaryEncoder));
        setDefaultCommand(run(() -> setSpeedDutyCycle(0)));
    }

    /**
     * Sets the voltage of the motor by using the SimpleMotorFeedForward for the flywheel to compute the required voltage for the desired velocity
     *
     * @param speed the speed to set the motor to, in RPM
     * @Note: This method is not used for the Flywheel LinearSystem (state-space).
     * To use FF there, a LinearSystem must be instantiated using LinearSystemId.identifyVelocitySystem(double kV, double kA) </p>
     * */
    public void setFFVoltageWithVelocity(double speed) {
        this.setSpeedVoltage(feedforward.calculate(speed));
    }

    /**
     * Gets the velocity of the flywheel (this is done by applying the motor-to-flywheel gearing)
     */
    public double getFlywheelVelocity() {
        return getVelocity() / kFlywheelGearing;
    }

    /** This method will be called once per scheduler run */
    @Override
    public void periodic() {
        super.periodic();
        //logger.logDouble("Flywheel setpoint velocity", Robot.firing.power * Math.PI * Robot.SHOOTER_LOSS_COMPENSATION);
        // Add any additional periodic logic here
    }
}
