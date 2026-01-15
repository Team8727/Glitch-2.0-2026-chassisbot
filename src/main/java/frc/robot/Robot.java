// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import Glitch.Lib.Controller.Controller;
import Glitch.Lib.NetworkTableLogger;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Drivetrain.CTRESwerveDrivetrain;
import frc.robot.Drivetrain.TunerConstants;
import frc.robot.controller.Driver1DefaultBindings;
import frc.robot.controller.ProjectileSolver;
import org.json.simple.parser.ParseException;
import org.littletonrobotics.urcl.URCL;

import java.io.IOException;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
  private double lastTime = Timer.getFPGATimestamp();
  private final CTRESwerveDrivetrain CTREDrivetrain = TunerConstants.createDrivetrain();
  private double deltaTime;
  private final CTRESwerveDrivetrain CTRDrivetrain = TunerConstants.createDrivetrain();
  public static ProjectileSolver.FiringSolution firing;
  private final Vision vision = new Vision();
//  private final LEDSubsystem m_ledSubsystem = LEDSubsystem.getInstance();
  private final NetworkTableLogger logger = new NetworkTableLogger("Robot");
  private final Autos autos = new Autos(CTRDrivetrain);
  private final Controller m_mainController = new Controller(Controller.Operator.MAIN); // Main controller
  private final Controller m_assistController = new Controller(Controller.Operator.ASSIST); // Assist controller



  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {
    CTREDrivetrain.setVision(vision);
    // Configure PathPlanner's AutoBuilder
    try {
      AutoBuilder.configure(
        () -> CTRDrivetrain.getState().Pose,
        CTRDrivetrain::resetPose,
        () -> CTRDrivetrain.getState().Speeds,
        (chassisSpeeds, driveFF) -> { // drive command
          final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage);

          // INVERT IF THINGS ARE GOING BACKWARDS
          // if (Robot.isRedAlliance()) {
          //   chassisSpeeds = new ChassisSpeeds(-chassisSpeeds.vxMetersPerSecond, -chassisSpeeds.vyMetersPerSecond, chassisSpeeds.omegaRadiansPerSecond);
          // }

          CTREDrivetrain.applyRequest(() ->
            drive.withVelocityX(-chassisSpeeds.vyMetersPerSecond) // Drive forward with negative Y (forward)
              .withVelocityY(-chassisSpeeds.vxMetersPerSecond) // Drive left with negative X (left)
              .withRotationalRate(-chassisSpeeds.omegaRadiansPerSecond) // Drive counterclockwise with negative X (left)
          );
        },
        new PPHolonomicDriveController(
          new PIDConstants(
            80,
            0,
            0),
          new PIDConstants(
            40,
            0,
            0)),
        RobotConfig.fromGUISettings(),
        Robot::isRedAlliance,
        // requirements
        CTREDrivetrain);
    } catch (IOException | ParseException e) {
      System.out.println("ERROR: Could not process pathPlanner config");
      throw new RuntimeException(e);
    }

    // Load autos into chooser and use SmartDashboard to publish
    autos.setupAutoChooser();
    SmartDashboard.putData("Auto choices", autos.getAutoChooser());

    // Set Up PathPlanner to "warm up" the pathPlanning system
    CommandScheduler.getInstance().schedule(PathfindingCommand.warmupCommand());

    // Log data to a log file using WPILib's DataLogManager
    DataLogManager.logNetworkTables(true);
    DataLogManager.start();

    // Start the URCL logger (logs REV SparkMaxes and SparkFlexes automatically on networkTables)
    URCL.start();
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
    logger.logDouble("voltage", RobotController.getInputVoltage());
    double now = Timer.getFPGATimestamp();
    deltaTime = now - lastTime;
    lastTime = now;


    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
  }

  /** This function is called periodically during disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link Autos} class. */
  @Override
  public void autonomousInit() {
    CommandScheduler.getInstance().cancelAll();
    m_mainController.clearBindings();
    m_assistController.clearBindings();
    autos.selectAuto(); //Only enable this if you want the robot to do stuff during autonomous
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    // This makes sure that autonomous stops running when teleop starts running.
    CommandScheduler.getInstance().cancelAll();

    m_mainController.applyBindings(
      new Driver1DefaultBindings(
        CTRDrivetrain,
        autos,
        m_mainController.getController()
      )
    );
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    firing = ProjectileSolver.solve(
      new Translation3d(
        CTRDrivetrain.getState().Pose.getX(),
        CTRDrivetrain.getState().Pose.getY(),
        0.3),
      new Translation3d(10, 4.5, 1.8),
      new Translation3d(
        CTRDrivetrain.getState().Speeds.vxMetersPerSecond,
        CTRDrivetrain.getState().Speeds.vyMetersPerSecond,
        0),
      -50);
    logger.logDouble("shooter vel", firing.power);
    logger.logDouble("shooter yaw", firing.yaw);
    logger.logDouble("shooter yaw radians", Math.toRadians(firing.yaw));
    logger.logDouble("shooter2 pitch", firing.pitch);
    logger.logBoolean("shooter2 valid", firing.isValid);
    logger.logPose3d("shooter2 position", new Pose3d(
      new Translation3d(
        CTRDrivetrain.getState().Pose.getX(),
        CTRDrivetrain.getState().Pose.getY(),
        0.3),
      new Rotation3d(0, Math.toRadians(firing.pitch), Math.toRadians(firing.yaw))));
    logger.logPose3d("target", new Pose3d(
      new Translation3d(10, 4.5, 1.8),
      new Rotation3d()));
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
