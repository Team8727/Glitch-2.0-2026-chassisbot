// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import Glitch.Lib.Controller.Controller;
import Glitch.Lib.LEDs.GlitchLEDPatterns;
import Glitch.Lib.NetworkTableLogger;
import com.pathplanner.lib.commands.PathfindingCommand;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Drivetrain.CTRESwerveDrivetrain;
import frc.robot.Drivetrain.TunerConstants;
import frc.robot.Subsystems.Indexer;
import frc.robot.Subsystems.IntakeRoller;
import frc.robot.Subsystems.LEDSubsystem;
import frc.robot.Subsystems.ShooterRoller;
import frc.robot.controller.Driver1DefaultBindings;
import frc.robot.controller.ProjectileSolver;
import org.littletonrobotics.urcl.URCL;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {

  private static final double SHOOTER_ANGLE_DEGREES = 60.0;
  private static final double SHOOTER_HEIGHT_METERS = 0.3;
  public static final double SHOOTER_FLYWHEEL_RADIUS_METERS = 0.0503555;
  private static final Translation3d BLUE_ALLIANCE_TARGET_3D = new Translation3d(4.626, 4.035, 1.8);
  private static final Translation3d RED_ALLIANCE_TARGET_3D = new Translation3d(11.915, 4.035, 1.8);
  public static final double SHOOTER_LOSS_COMPENSATION = 2;
  public static boolean SHOOT_POWER_OVERRIDE = false;

  private Translation3d target;

  public static ProjectileSolver.FiringSolution firing;
  private double lastTime = Timer.getFPGATimestamp();
  private double deltaTime;

  // Used for drivetrain oscillation command (wiggling)
  public static Rotation2d referenceRotation = new  Rotation2d();

  private final ShooterRoller shooterRoller = new ShooterRoller();

  private final NetworkTableLogger logger = new NetworkTableLogger("Robot");
  private final CTRESwerveDrivetrain CTREDrivetrain = TunerConstants.createDrivetrain();
  private final Vision vision = new Vision();
  private final IntakeRoller intakeRoller = new IntakeRoller();
  private final Indexer indexer = new Indexer();
  private final LEDSubsystem leds = LEDSubsystem.getInstance();
  private final Autos autos = new Autos(CTREDrivetrain, indexer, shooterRoller, intakeRoller, leds);
  private final Controller mainController = new Driver1DefaultBindings(autos, CTREDrivetrain, intakeRoller, indexer, shooterRoller, leds);


  public Servo pinServo;


  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {
    CTREDrivetrain.setVision(vision);
    // Set Up PathPlanner to "warm up" the pathPlanning system
    CommandScheduler.getInstance().schedule(PathfindingCommand.warmupCommand());

    // Log data to a log file using WPILib's DataLogManager
    DataLogManager.logNetworkTables(true);
    DataLogManager.start();

    // Start the URCL logger (logs REV SparkMaxes and SparkFlexes automatically on networkTables)
    URCL.start();

    SmartDashboard.putNumber("Shooter power", 0);

    // Used by oscillation command
    addPeriodic(() -> referenceRotation = CTREDrivetrain.getState().Pose.getRotation().minus(Rotation2d.fromDegrees(180)), 0.01);

    addPeriodic(() -> {
      if (measurementCount == 1) {
        confidenceNow = lowConfidence;
      }
      if (measurementCount == 2) {
        confidenceNow = reasonableConfidence;
      }
      if (measurementCount > 2) {
        confidenceNow = highConfidence;
      }},
      0.5);

    // Setup zones
//    new ZoneController(
//            CTREDrivetrain,
//            new Rectangle(1,1,1,1),
//            () -> intakePivot.setPosition(IntakePivot.IntakePosition.MID.getDegrees()));

    leds.initializeLEDS(0);
    pinServo = new Servo(1);
    pinServo.setAngle(0);
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */

  int measurementCount; // Number of vision measurements of each tag per camera
  LEDPattern lowConfidence = GlitchLEDPatterns.ripple(LEDPattern.solid(Color.kYellow), 10, 20);
  LEDPattern reasonableConfidence = GlitchLEDPatterns.ripple(GlitchLEDPatterns.ace, 10, 20);
  LEDPattern highConfidence = GlitchLEDPatterns.ripple(GlitchLEDPatterns.funGradient, 20, 30);
  LEDPattern confidenceNow;
  double ledRefreshTime = 0.5; // Time in seconds to refresh the LED pattern

  @Override
  public void robotPeriodic() {
    target = isRedAlliance() ? RED_ALLIANCE_TARGET_3D : BLUE_ALLIANCE_TARGET_3D;
    logger.logDouble("voltage", RobotController.getInputVoltage());
    double now = Timer.getFPGATimestamp();
    deltaTime = now - lastTime;
    lastTime = now;
    vision.logCameraPoses(CTREDrivetrain.getState().Pose);


    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();

    // Vision fusion is done in drivetrain.periodic(); read the most recent fused count here.
    measurementCount = CTREDrivetrain.getLastVisionMeasurementCount();

    Translation3d shooterFieldPosition = new Translation3d(
            CTREDrivetrain.getState().Pose.getX(),
            CTREDrivetrain.getState().Pose.getY(),
            SHOOTER_HEIGHT_METERS);

    Translation3d drivetrainFOCVelocity = new Translation3d(
            CTREDrivetrain.getState().Speeds.vxMetersPerSecond,
            CTREDrivetrain.getState().Speeds.vyMetersPerSecond,
            0).rotateBy(new Rotation3d(CTREDrivetrain.getState().Pose.getRotation()));// rotate by robot rotation

    firing = ProjectileSolver.solve(
            shooterFieldPosition,
            target,
            drivetrainFOCVelocity,// rotate by robot rotation
            SHOOTER_ANGLE_DEGREES);

    logger.logDouble("shooter vel", firing.power);
    logger.logDouble("[OLD] Flywheel setpoint velocity", Robot.firing.power * Math.PI * Robot.SHOOTER_LOSS_COMPENSATION);
    logger.logDouble("[NEW] Flywheel setpoint velocity", 2 * ((Robot.firing.power) / (Math.PI * 2 * Robot.SHOOTER_FLYWHEEL_RADIUS_METERS))); // Convert m/s to rpm)
    logger.logDouble("shooter yaw", firing.yaw);
    logger.logDouble("shooter yaw radians", Math.toRadians(firing.yaw));
    logger.logDouble("shooter2 pitch", firing.pitch);
    logger.logBoolean("shooter2 valid", firing.isValid);
    logger.logDouble("shooter2 horizontal distance", firing.horizontalDistance);

    logger.logPose3d("shooter2 position", new Pose3d(
            shooterFieldPosition,
            new Rotation3d(0, Math.toRadians(firing.pitch), Math.toRadians(firing.yaw))));

    logger.logPose3d("target", new Pose3d(
            target,
            new Rotation3d()));

    logger.logChassisSpeeds("world velocity", new ChassisSpeeds(firing.worldVel.getX(), firing.worldVel.getY(), 0));

    logger.logInt("vision measurement count", measurementCount);
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    leds.start();
    pinServo.setAngle(0);
  }

  /** This function is called periodically during disabled. */
  @Override
  public void disabledPeriodic() {}

  @Override
  public void autonomousInit() {
    CommandScheduler.getInstance().cancelAll();

    // Get the selected autonomous command from the Autos class
    Command autoCommand = autos.getAutonomousCommand();
    if (autoCommand != null) {
      CommandScheduler.getInstance().schedule(autoCommand);
    }

    leds.autoInit();
    pinServo.setAngle(180);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    leds.leftSide.setBase(confidenceNow);
    leds.pip.setBase(confidenceNow);
    leds.rightSide.setBase(confidenceNow);
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    // This makes sure that autonomous stops running when teleop starts running.
    CommandScheduler.getInstance().cancelAll();

    shooterRoller.m_loop.reset(VecBuilder.fill(Units.rotationsPerMinuteToRadiansPerSecond(shooterRoller.getVelocity())));

    leds.teleopInit();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    leds.leftSide.setBase(confidenceNow);
    leds.pip.setBase(confidenceNow);
    leds.rightSide.setBase(confidenceNow);
  }

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
  }

  public static boolean isRedAlliance() {
    return DriverStation.getAlliance().orElse(Alliance.Blue).equals(Alliance.Red);
  }
}
