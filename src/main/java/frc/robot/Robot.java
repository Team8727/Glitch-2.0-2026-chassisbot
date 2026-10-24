package frc.robot;

import com.pathplanner.lib.commands.PathfindingCommand;
import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.autos.AutoChooser;
import frc.robot.commands.DriveCommands;
import frc.robot.controls.Controls;
import frc.robot.shooting.ProjectileSolver;
import frc.robot.shooting.ProjectileSolver.FiringSolution;
import frc.robot.shooting.Targeting;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.TunerConstants;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.leds.LED;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.drivetrain.Telemetry;
import frc.robot.subsystems.vision.Vision;
import edu.wpi.first.wpilibj.Servo;
import frc.lib.util.NetworkTableLogger;
import org.littletonrobotics.urcl.URCL;

/**
 * Main robot class for Team 8727 Glitch 2.0 — 2026 REBUILT.
 * This is the THOR rewrite with improved structure, regression-based shooting,
 * auto-alignment, and enhanced pose estimation.
 */
public class Robot extends TimedRobot {

  // ========== Firing solution (computed every cycle) ==========

  /** Current firing solution, pre-filled with a safe default for null-safety. */
  public static FiringSolution firing = ProjectileSolver.emptySolution();

  /** Reference rotation for oscillation command. */
  public static Rotation2d referenceRotation = new Rotation2d();

  // ========== Subsystems (instantiated once) ==========

  private final Drivetrain drivetrain = TunerConstants.createDrivetrain();
  private final Shooter shooter = new Shooter();
  private final Intake intake = new Intake();
  private final Indexer indexer = new Indexer();
  private final Vision vision = new Vision();
  private final LED leds = LED.instance;

  private final NetworkTableLogger logger = new NetworkTableLogger("Robot");

  // ========== Autonomous pin servo (PWM port 1) ==========

  /** Used to release a physical pin at the start of autonomous. */
  private final Servo pinServo = new Servo(1);

  // ========== Auto ==========

  private final AutoChooser autoChooser;

  /** Constructor — runs at robot boot. */
  public Robot() {
    // Wire vision into drivetrain for pose fusion
    drivetrain.setVision(vision);

    // Build controls (binds buttons to commands — field not stored, side-effect only)
    new Controls(drivetrain, shooter, intake, indexer);

    // Initialize drive tuning sliders on dashboard
    DriveCommands.initDashboardTuning();

    // Register telemetry for dashboard and sim visualization
    var telemetry = new Telemetry(TunerConstants.kSpeedAt12Volts.in(edu.wpi.first.units.Units.MetersPerSecond));
    drivetrain.registerTelemetry(telemetry::telemeterize);

    // Build auto chooser (dashboard-selectable routines).
    // Pass the driver controller so autos can be cycled while disabled using bumpers.
    autoChooser = new AutoChooser(drivetrain, shooter, intake, indexer, new CommandXboxController(0));

    // Start with pin retracted
    pinServo.setAngle(0);

    // Warm up PathPlanner
    CommandScheduler.getInstance().schedule(PathfindingCommand.warmupCommand());

    // ========== Logging ==========

    DataLogManager.logNetworkTables(true);
    DataLogManager.start();

    URCL.start();

    DogLog.setOptions(
        new DogLogOptions()
            .withCaptureConsole(true)
            .withCaptureDs(true)
            .withCaptureNt(true)
            .withLogEntryQueueCapacity(1000)
            .withLogExtras(true)
            .withNtPublish(true));
    DogLog.setEnabled(true);
    DogLog.log("RIO/SerialNumber", RobotController.getSerialNumber());

    // ========== Dashboard ==========

    SmartDashboard.putNumber("Shooter power override", 0);

    // Periodic oscillation reference update
    addPeriodic(
        () -> referenceRotation =
            drivetrain.getState().Pose.getRotation().minus(Rotation2d.fromDegrees(180)),
        0.04);
  }

  // ========== Periodic ==========

  @Override
  public void robotPeriodic() {
    try {
      robotPeriodicInner();
    } catch (Exception e) {
      logger.logString("lastError", e.getClass().getSimpleName() + ": " + e.getMessage());
      DogLog.log("Robot/PeriodicError", e.toString());
    }
  }

  private void robotPeriodicInner() {
    // Log battery voltage
    logger.logDouble("voltage", RobotController.getInputVoltage());

    // Publish camera poses for dashboard visualization
    vision.logCameraPoses(drivetrain.getState().Pose);

    // Run command scheduler (polls buttons, runs commands, runs subsystem periodic)
    CommandScheduler.getInstance().run();

    // ========== Odometry diagnostics ==========

    logger.logBoolean("odometry/valid", drivetrain.isOdometryValid());
    try {
      logger.logDouble("odometry/frequency", drivetrain.getOdometryFrequency());
      logger.logDouble("odometry/timestamp", drivetrain.getState().Timestamp);
      logger.logInt("odometry/successfulDaqs", drivetrain.getState().SuccessfulDaqs);
    } catch (Exception e) {
      logger.logString("odometry/error", e.getMessage());
    }

    // ========== Compute firing solution every cycle ==========
    // Always target the HUB — ZR shoots for the hub, and A aligns to it.

    Translation3d target = Targeting.getHubTargetTranslation();

    Translation3d shooterPosition = new Translation3d(
        drivetrain.getState().Pose.getX(),
        drivetrain.getState().Pose.getY(),
        Constants.kShooterHeightMeters);

    // Robot velocity in field frame (rotate chassis speeds by robot heading)
    var speeds = drivetrain.getState().Speeds;
    Translation3d robotVelocity = new Translation3d(
        speeds.vxMetersPerSecond,
        speeds.vyMetersPerSecond,
        0)
        .rotateBy(new Rotation3d(drivetrain.getState().Pose.getRotation()));

    firing = ProjectileSolver.solve(
        shooterPosition,
        target,
        robotVelocity,
        Constants.kShooterAngleDegrees);

    // ========== Log firing data ==========

    logger.logDouble("shooter/requiredPowerMPS", firing.power);
    logger.logDouble("shooter/yawDeg", firing.yaw);
    logger.logDouble("shooter/yawRad", Math.toRadians(firing.yaw));
    logger.logDouble("shooter/pitchDeg", firing.pitch);
    logger.logBoolean("shooter/valid", firing.isValid);
    logger.logDouble("shooter/horizontalDistance", firing.horizontalDistance);

    logger.logPose3d("shooter/aimPose", new Pose3d(
        shooterPosition,
        new Rotation3d(0, Math.toRadians(firing.pitch), Math.toRadians(firing.yaw))));

    logger.logPose3d("shooter/targetPose", new Pose3d(target, new Rotation3d()));

    // ========== Vision confidence ==========

    int measurementCount = drivetrain.getLastVisionMeasurementCount();
    leds.updateVisionConfidence(measurementCount);
    logger.logInt("vision/measurementCount", measurementCount);

    // ========== LED updates ==========

    // Shooting progress bar — live RPM from the shooter, real target from projectile solver
    double solverSlipComp = Math.max(0.0, Math.min(0.5, SmartDashboard.getNumber("Shooter Slip Comp", 0.15)));
    double shooterRPS = shooter.getFlywheelVelocity();
    double targetFlywheelRPS = Robot.firing.power / (Math.PI * Constants.kShooterFlywheelDiameterMeters) / (1.0 - solverSlipComp);
    if (shooterRPS > 1.0) {
        leds.shootPatterns(shooterRPS, targetFlywheelRPS);
    } else {
        leds.endCommand();
    }

    // Intake LED — track from actual motor current (handles toggle-off correctly)
    leds.setIntaking(intake.getOutputCurrent() > 0.5);

    // Publish zone awareness
    var zone = Targeting.getZone(drivetrain.getState().Pose);
    logger.logString("zone", zone.name());

    // Publish vision processing status (true if cameras report tags this cycle)
    logger.logBoolean("vision/processing", measurementCount > 0);
    // Camera read-error counter — spikes when a Pi browns out
    logger.logInt("vision/cameraErrors", vision.getCameraErrorCount());

    // Publish look-ahead distance (for dashboard display)
    var chassisSpeeds = new ChassisSpeeds(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond);
    logger.logDouble("shooter/lookaheadDistance", Targeting.getLookaheadDistance(drivetrain.getState().Pose, chassisSpeeds));

    // Publish the currently selected aim target for the dashboard
    logger.logString("shooter/targetName", Targeting.getCurrentTargetName());

    // ========== Continuous dashboard telemetry (even when not shooting) ==========

    // Target RPM from the projectile solver (matches what WaitForFlywheelSpeed uses)
    double solverFlywheelRPS = Robot.firing.power / (Math.PI * Constants.kShooterFlywheelDiameterMeters);
    double solverTargetRPM = solverFlywheelRPS / (1.0 - solverSlipComp) * 60.0; // slip comp + convert RPS→RPM
    SmartDashboard.putNumber("Shooter RPM Target", solverTargetRPM);
    SmartDashboard.putNumber("Shooter Amps", shooter.getOutputCurrent());
    SmartDashboard.putNumber("Indexer Amps", indexer.getOutputCurrent());
    SmartDashboard.putNumber("Total Amps", shooter.getOutputCurrent() + indexer.getOutputCurrent() + intake.getOutputCurrent());
  }

  // ========== Mode lifecycle ==========

  @Override
  public void disabledInit() {
    leds.start();
    pinServo.setAngle(0);
    Targeting.resetTarget(); // Reset aim target to HUB each match
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void autonomousInit() {
    CommandScheduler.getInstance().cancelAll();

    leds.autoInit();

    // Deploy pin to release mechanism
    pinServo.setAngle(180);

    Command autoCommand = autoChooser.getSelected();
    if (autoCommand != null) {
      CommandScheduler.getInstance().schedule(autoCommand);
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    CommandScheduler.getInstance().cancelAll();

    // Reset state-space loop
    shooter.m_loop.reset(VecBuilder.fill(shooter.getVelocity()));

    leds.teleopInit();
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}

  // ========== Helpers ==========

  public static boolean isRedAlliance() {
    return DriverStation.getAlliance().orElse(Alliance.Blue).equals(Alliance.Red);
  }
}