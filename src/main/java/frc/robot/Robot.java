// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import Glitch.Lib.Controller.Controller;
import Glitch.Lib.LEDS.GlitchLEDPatterns;
import Glitch.Lib.NetworkTableLogger;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Commands.Shoot;
import frc.robot.Drivetrain.CTRESwerveDrivetrain;
import frc.robot.Drivetrain.TunerConstants;
import frc.robot.Subsystems.LEDSubsystem2026;
import frc.robot.Subsystems.*;
import frc.robot.controller.CTReSwerveControls;
import frc.robot.controller.Driver1DefaultBindings;
import frc.robot.controller.ProjectileSolver;

import frc.robot.controller.ZoneController;
import org.json.simple.parser.ParseException;
import org.littletonrobotics.urcl.URCL;

import javax.naming.Name;
import java.awt.*;
import java.io.IOException;

import static edu.wpi.first.wpilibj2.command.Commands.run;
import static edu.wpi.first.wpilibj2.command.Commands.runOnce;
import static frc.robot.controller.CTReSwerveControls.MaxAngularRate;
import static frc.robot.controller.CTReSwerveControls.MaxSpeed;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {

  private static final double SHOOTER_ANGLE_DEGREES = 60.0;
  private static final double SHOOTER_HEIGHT_METERS = 0.3;
  private static final Translation3d BLUE_ALLIANCE_TARGET_3D = new Translation3d(4.626, 4.035, 1.8);
  private static final Translation3d RED_ALLIANCE_TARGET_3D = new Translation3d(11.915, 4.035, 1.8);
  private Translation3d target;

  public static ProjectileSolver.FiringSolution firing;
  private double lastTime = Timer.getFPGATimestamp();
  private double deltaTime;

  private final ShooterRoller shooterRoller = new ShooterRoller();
  private final ShooterRollerFollower shooterRollerFollower = new ShooterRollerFollower();

  private final NetworkTableLogger logger = new NetworkTableLogger("Robot");
  private final CTRESwerveDrivetrain CTREDrivetrain = TunerConstants.createDrivetrain();
  private final Vision vision = new Vision();
  private final Controller m_mainController = new Controller(Controller.Operator.MAIN); // Main controller
  private final Controller m_assistController = new Controller(Controller.Operator.ASSIST); // Assist controller
  private final IntakePivot intakePivot = new IntakePivot();
  private final IntakeRoller intakeRoller = new IntakeRoller();
  private final Indexer indexer = new Indexer();
  private final Spindexer spindexer = new Spindexer();
  private final Autos autos = new Autos(CTREDrivetrain, indexer, shooterRoller, spindexer, intakePivot, intakeRoller);
  private final LEDSubsystem2026 m_leds = LEDSubsystem2026.getInstance();

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

    m_mainController.applyBindings(
            new Driver1DefaultBindings(
                    m_mainController.getController(),
                    autos,
                    CTREDrivetrain,
                    spindexer,
                    intakePivot,
                    intakeRoller,
                    indexer,
                    shooterRoller
            )
    );

    SmartDashboard.putNumber("Shooter power", 0);
    // Setup zones
//    new ZoneController(
//            CTREDrivetrain,
//            new Rectangle(1,1,1,1),
//            () -> intakePivot.setPosition(IntakePivot.IntakePosition.MID.getDegrees()));
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
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
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    m_leds.setAll(GlitchLEDPatterns.purple);
    m_leds.initializeLEDS(0);
  }

  /** This function is called periodically during disabled. */
  @Override
  public void disabledPeriodic() {}

  @Override
  public void autonomousInit() {
    CommandScheduler.getInstance().cancelAll();
    m_mainController.clearBindings();
    m_assistController.clearBindings();
    
    // Get the selected autonomous command from the Autos class
    Command autoCommand = autos.getAutonomousCommand();
    if (autoCommand != null) {
      CommandScheduler.getInstance().schedule(autoCommand);
    }

    m_leds.setAll(GlitchLEDPatterns.fire(GlitchLEDPatterns.funGradient, Color.kGreen));
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    // This makes sure that autonomous stops running when teleop starts running.
    CommandScheduler.getInstance().cancelAll();
    intakeRoller.stickySetDuty(0);

    m_leds.returnAllToBase();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

    if (firing.isValid) {
      m_leds.setAll(GlitchLEDPatterns.ripple(GlitchLEDPatterns.funGradient, 5, 1), 0.3);
    }
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
