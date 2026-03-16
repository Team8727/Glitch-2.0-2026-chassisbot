package frc.robot.Subsystems;

import Glitch.Lib.BaseMechanisms.Roller;
import Glitch.Lib.Motors.SparkMaxMotor;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

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
                .pid(.07, 0, 0); //TODO: Tune PID values
    }

    private double kS = 0; // Find using SysID
    private double kV = 0;
    private double kA = 0;
    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(kS, kV, kA);

    public SysIdRoutine sysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                    null,
                    null,
                    null,
                    state -> logger.log("SysIdFlywheel_State", state.toString())
            ),
            new SysIdRoutine.Mechanism(
                    (voltage) -> setSpeedVoltage(voltage.in(Volts)),
                    null, // No log consumer, since data is recorded by URCL
                    this
            )
    );

    public ShooterRoller() {
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
