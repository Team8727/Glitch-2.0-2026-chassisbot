package frc.robot.subsystems.drivetrain;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import dev.doglog.DogLog;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionFallback;
import java.io.IOException;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;
import org.json.simple.parser.ParseException;
import frc.lib.util.NetworkTableLogger;

public class Drivetrain extends TunerConstants.TunerSwerveDrivetrain implements Subsystem {
    private Vision m_Vision = null;
    private int m_lastVisionMeasurementCount = 0;
    private final VisionFallback m_visionFallback = new VisionFallback();
    private final NetworkTableLogger logger = new NetworkTableLogger("Drivetrain");
    private final Field2d m_field = new Field2d();
    
    private static final double kSimLoopPeriod = 0.004;
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;
    
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
    private boolean m_hasAppliedOperatorPerspective = false;
    
    private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
    private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();
    
    private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(
        new SysIdRoutine.Config(null, Volts.of(4), null,
            state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())),
        new SysIdRoutine.Mechanism(
            output -> setControl(m_translationCharacterization.withVolts(output)), null, this));
    
    private final SysIdRoutine m_sysIdRoutineSteer = new SysIdRoutine(
        new SysIdRoutine.Config(null, Volts.of(7), null,
            state -> SignalLogger.writeString("SysIdSteer_State", state.toString())),
        new SysIdRoutine.Mechanism(
            volts -> setControl(m_steerCharacterization.withVolts(volts)), null, this));
    
    private final SysIdRoutine m_sysIdRoutineRotation = new SysIdRoutine(
        new SysIdRoutine.Config(
            Volts.of(Math.PI / 6).per(Second), Volts.of(Math.PI), null,
            state -> SignalLogger.writeString("SysIdRotation_State", state.toString())),
        new SysIdRoutine.Mechanism(
            output -> {
                setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
                SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
            }, null, this));
    
    private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineRotation;
    
    public Drivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, modules);
        if (Utils.isSimulation()) startSimThread();

        SmartDashboard.putData("Field", m_field);
        
        try {
            AutoBuilder.configure(
                () -> getState().Pose,
                this::resetPose,
                () -> getState().Speeds,
                (cs, driveFF) -> CommandScheduler.getInstance().schedule(
                    applyRequest(() -> new SwerveRequest.ApplyRobotSpeeds().withSpeeds(cs))),
                new PPHolonomicDriveController(
                    new PIDConstants(10, 0, 0),
                    new PIDConstants(7, 0, 0)),
                RobotConfig.fromGUISettings(),
                Robot::isRedAlliance);
        } catch (IOException | ParseException e) {
            throw new RuntimeException(e);
        }
    }
    
    public Command applyRequest(Supplier<SwerveRequest> request) {
        return run(() -> this.setControl(request.get()));
    }
    
    public void setVision(Vision vision) { this.m_Vision = vision; }
    
    public int getLastVisionMeasurementCount() { return m_lastVisionMeasurementCount; }
    
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.quasistatic(direction);
    }
    
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.dynamic(direction);
    }
    
    @Override
    public void periodic() {
        if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                setOperatorPerspectiveForward(
                    allianceColor == Alliance.Red
                        ? kRedAlliancePerspectiveRotation
                        : kBlueAlliancePerspectiveRotation);
                m_hasAppliedOperatorPerspective = true;
            });
        }
        
        if (m_Vision != null) {
            m_Vision.periodic();
            
            Pose2d reference = this.getState().Pose;
            List<Vision.Measurement> measurements = m_Vision.drainMeasurements(reference);
            m_lastVisionMeasurementCount = measurements.size();
            
            for (Vision.Measurement m : measurements) {
                double dist = Double.isNaN(m.avgDistanceMeters())
                    ? Constants.kVisionUnknownDistanceMeters
                    : m.avgDistanceMeters();
                double linearStdDev = Constants.kVisionLinearStdDevBaseline * dist * dist;
                double angularStdDev = Constants.kVisionAngularStdDevBaseline * dist * dist;
                
                addVisionMeasurement(
                    m.pose(), m.timestampSeconds(),
                    VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev));
            }
            
            if (DogLog.isEnabled() && !measurements.isEmpty()) {
                double avgDist = measurements.stream()
                    .filter(m -> !Double.isNaN(m.avgDistanceMeters()))
                    .mapToDouble(Vision.Measurement::avgDistanceMeters)
                    .average().orElse(-1.0);
                DogLog.log("Vision/MeasurementCount", measurements.size());
                DogLog.log("Vision/AvgTagDistance", avgDist);
            }
            
            logger.logInt("vision/measurementCount", measurements.size());
        } else {
            m_lastVisionMeasurementCount = 0;
        }

        // Run vision fallback — publishes confidence to dashboard
        // When vision is down, this provides a dead-reckoned pose estimate
        ChassisSpeeds speeds = getState().Speeds;
        var chassisSpeeds = new ChassisSpeeds(
            speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond);
        Pose2d fallbackPose = m_visionFallback.update(
            m_lastVisionMeasurementCount, chassisSpeeds, getState().Pose);
        logger.logString("vision/fallbackStatus", m_visionFallback.getConfidence().name());

        // Update Field2d for sim GUI and dashboard
        m_field.setRobotPose(getState().Pose);
    }
    
    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();
        m_simNotifier = new Notifier(() -> {
            double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }
    
    @Override
    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds));
    }
    
    @Override
    public void addVisionMeasurement(
        Pose2d visionRobotPoseMeters, double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs
    ) {
        super.addVisionMeasurement(
            visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds), visionMeasurementStdDevs);
    }
    
    @Override
    public Optional<Pose2d> samplePoseAt(double timestampSeconds) {
        return super.samplePoseAt(Utils.fpgaToCurrentTime(timestampSeconds));
    }
}