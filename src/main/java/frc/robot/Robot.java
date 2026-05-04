// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import Glitch.Lib.Controller.Controller;
import Glitch.Lib.LEDs.AbstractLEDS;
import Glitch.Lib.LEDs.GlitchLEDPatterns;
import Glitch.Lib.NetworkTableLogger;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Drivetrain.CTRESwerveDrivetrain;
import frc.robot.Drivetrain.TunerConstants;
import frc.robot.Subsystems.Indexer;
import frc.robot.Subsystems.IntakeRoller;
import frc.robot.Subsystems.LEDSubsystem;
import frc.robot.Subsystems.LEDTraining;
import frc.robot.Subsystems.ShooterRoller;
import frc.robot.Subsystems.LEDTraining.MyTestPattern;
import frc.robot.controller.Driver1DefaultBindings;
import frc.robot.controller.ProjectileSolver;

import static edu.wpi.first.units.Units.Microseconds;
import static edu.wpi.first.units.Units.Seconds;

import org.littletonrobotics.urcl.URCL;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {

  private static final double SHOOTER_ANGLE_DEGREES = 73.0;
  private static final double SHOOTER_HEIGHT_METERS = 0.3;
  public static final double SHOOTER_FLYWHEEL_DIAMETER_METERS = 0.0889;
  private static final Translation3d BLUE_ALLIANCE_TARGET_3D = new Translation3d(4.626, 4.035, 1.8);
  private static final Translation3d RED_ALLIANCE_TARGET_3D = new Translation3d(11.915, 4.035, 1.8);

  private Translation3d target;

  public static ProjectileSolver.FiringSolution firing;

  // Used for drivetrain oscillation command (wiggling)
  public static Rotation2d referenceRotation = new  Rotation2d();

  private final ShooterRoller shooterRoller = new ShooterRoller();

  private final NetworkTableLogger logger = new NetworkTableLogger("Robot");
  private final CTRESwerveDrivetrain CTREDrivetrain = TunerConstants.createDrivetrain();
  private final Vision vision = new Vision();
  private final IntakeRoller intakeRoller = new IntakeRoller();
  private final Indexer indexer = new Indexer();
  private final LEDTraining leds = LEDTraining.getInstance();
  private final Autos autos = new Autos(CTREDrivetrain, indexer, shooterRoller, intakeRoller);
  private final Controller mainController = new Driver1DefaultBindings(autos, CTREDrivetrain, intakeRoller, indexer, shooterRoller);
  public static final Field2d field = new Field2d();


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

    SmartDashboard.putData("Field", field);

    // Start the URCL logger (logs REV SparkMaxes and SparkFlexes automatically on networkTables)
    URCL.start();

    // Used by oscillation command
    addPeriodic(() -> referenceRotation = CTREDrivetrain.getState().Pose.getRotation().minus(Rotation2d.fromDegrees(180)), 0.04); // Update period should be a multiple of the loop time: 0.02 seconds

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
//            Commands.runOnce(() -> intakePivot.setPosition(IntakePivot.IntakePosition.MID.getDegrees())));

    leds.initializeLEDS(0);
    pinServo = new Servo(1);
    pinServo.setAngle(0);
    PathPlannerLogging.setLogActivePathCallback((poses) -> {
      field.getObject("Path").setPoses(poses);
    });
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
    logger.logDouble("Flywheel setpoint velocity", (Robot.firing.power * (24.0 /15)) / (Math.PI * Robot.SHOOTER_FLYWHEEL_DIAMETER_METERS)); // rpm to rps
    logger.logDouble("shooter yaw", firing.yaw);
    logger.logDouble("shooter yaw radians", Math.toRadians(firing.yaw));
    logger.logBoolean("shooter2 valid", firing.isValid);
    logger.logDouble("shooter2 horizontal distance", firing.horizontalDistance);

//    logger.logPose3d("shooter2 position", new Pose3d(
//            shooterFieldPosition,
//            new Rotation3d(0, Math.toRadians(firing.pitch), Math.toRadians(firing.yaw))));

//    logger.logPose3d("target", new Pose3d(
//            target,
//            new Rotation3d()));

    logger.logChassisSpeeds("world velocity", new ChassisSpeeds(firing.worldVel.getX(), firing.worldVel.getY(), 0));

    logger.logInt("vision measurement count", measurementCount);

    // leds.ledStrip.setPattern(LEDPattern.solid(leds.stripColor(Microseconds.of(AbstractLEDS.getTime()).in(Seconds))));
    // LEDPattern pattern = leds.new MyTestPattern();
    LEDPattern pattern = leds.twoDPattern;
    leds.ledStrip.setPattern(pattern);
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
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
    pinServo.setAngle(125);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    // This makes sure that autonomous stops running when teleop starts running.
    CommandScheduler.getInstance().cancelAll();
    pinServo.setAngle(125);

    shooterRoller.m_loop.reset(VecBuilder.fill(Units.rotationsPerMinuteToRadiansPerSecond(shooterRoller.getVelocity())));
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

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
