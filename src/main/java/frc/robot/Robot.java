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
import frc.robot.autos.AutoChooser;
import frc.robot.controls.Controls;
import frc.robot.shooting.ProjectileSolver;
import frc.robot.shooting.ProjectileSolver.FiringSolution;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.TunerConstants;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.leds.LED;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.vision.Vision;
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

  // ========== Controllers & Auto ==========

  private final Controls controls;
  private final AutoChooser autoChooser;

  /** Constructor — runs at robot boot. */
  public Robot() {
    // Wire vision into drivetrain for pose fusion
    drivetrain.setVision(vision);

    // Build controls (binds buttons to commands)
    controls = new Controls(drivetrain, shooter, intake, indexer);

    // Build auto chooser (dashboard-selectable routines)
    autoChooser = new AutoChooser(drivetrain, shooter, intake, indexer);

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
    // Log battery voltage
    logger.logDouble("voltage", RobotController.getInputVoltage());

    // Publish camera poses for dashboard visualization
    vision.logCameraPoses(drivetrain.getState().Pose);

    // Run command scheduler (polls buttons, runs commands, runs subsystem periodic)
    CommandScheduler.getInstance().run();

    // ========== Compute firing solution every cycle ==========

    Translation3d target = isRedAlliance()
        ? Constants.kRedHubTarget
        : Constants.kBlueHubTarget;

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

    logger.logPose3d("shooter/target", new Pose3d(target, new Rotation3d()));

    logger.logChassisSpeeds("shooter/worldVelocity", new ChassisSpeeds(
        firing.worldVel.getX(), firing.worldVel.getY(), 0));

    // ========== Vision confidence ==========

    int measurementCount = drivetrain.getLastVisionMeasurementCount();
    leds.updateVisionConfidence(measurementCount);
    logger.logInt("vision/measurementCount", measurementCount);
  }

  // ========== Mode lifecycle ==========

  @Override
  public void disabledInit() {
    leds.start();
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void autonomousInit() {
    CommandScheduler.getInstance().cancelAll();

    leds.autoInit();

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