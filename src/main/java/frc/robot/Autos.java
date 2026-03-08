// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import Glitch.Lib.NetworkTableLogger;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Commands.Shoot;
import frc.robot.Drivetrain.CTRESwerveDrivetrain;
import frc.robot.Drivetrain.TunerConstants;
import frc.robot.Subsystems.*;
import org.json.simple.parser.ParseException;

import java.io.IOException;
import java.util.LinkedHashMap;

import static edu.wpi.first.wpilibj2.command.Commands.*;

public class Autos extends SubsystemBase {
  //  private final LEDSubsystem m_ledSubsystem = ;
  private final LinkedHashMap<String, PathPlannerPath> paths = new LinkedHashMap<>();
  private final CTRESwerveDrivetrain CTREDrivetrain;
  private final SendableChooser<String> autoChooser = new SendableChooser<>();
  private final NetworkTableLogger logger = new NetworkTableLogger(this.getName());
  private final Indexer indexer;
  private final ShooterRollers shooterRollers;
  private final Spindexer spindexer;
  private final IntakePivot intakePivot;
  private final IntakeRoller intakeRoller;

  private static final Translation2d fieldCenter = new Translation2d(8.770, 4.026); // meters

  /**
   * Creates a new Autos.
   */
  public Autos(CTRESwerveDrivetrain CTREDrivetrain, Indexer indexer, ShooterRollers shooterRollers, Spindexer spindexer, IntakePivot intakePivot, IntakeRoller intakeRoller) {
    this.CTREDrivetrain = CTREDrivetrain;
    this.indexer = indexer;
    this.shooterRollers = shooterRollers;
    this.spindexer = spindexer;
    this.intakePivot = intakePivot;
    this.intakeRoller = intakeRoller;

    loadPaths();
  }

  /**
   * Loads the paths from the specified path files.
   * Example:
   * <pre>
   *   loadPath("Path-Name"); </pre>
   */
  private void loadPaths() {
    loadPath("bareMinimum");
    loadPath("StartRight-Outpost");
    loadPath("Final plan 4.1");
    loadPath("main bump to");
    loadPath("main bump back");
    loadPath("main balls");
  }

  /**
   * Loads a specific path from the given path name.
   *
   * @param pathName The name of the path file to load.
   */
  private void loadPath(String pathName) {
    try {
      paths.put(pathName, PathPlannerPath.fromPathFile(pathName));
    } catch (IOException | ParseException e) {
      e.printStackTrace();
    }
  }

  /**
   * Sets up the auto chooser with different autonomous options.
   * Example:
   * <pre>
   *   autoChooser.setDefaultOption("Path-Name", "Path_Function()");
   *   autoChooser.addOption("Path-Name", "Path_Function()"); </pre>
   */
  public void setupAutoChooser() {
    autoChooser.setDefaultOption("BareMinimum", "bareMinimum()");
    autoChooser.addOption("StartRight-Outpost", "StartRightOutpost()");
    autoChooser.addOption("Final plan 4.1", "finalPath()");
  }

  /**
   * runs the autonomous command based on the selected option in the auto chooser.
   * <pre>
   *   autoChooser.setDefaultOption("Path-Name", "Path_Function()");
   *   autoChooser.addOption("Path-Name", "Path_Function()"); </pre>
   */
  public void selectAuto() {
    if (autoChooser.getSelected().equals("bareMinimum()")) {
      CommandScheduler.getInstance().schedule(bareMinimum());
    } else if (autoChooser.getSelected().equals("StartRightOutpost()")) {
      CommandScheduler.getInstance().schedule(StartRightOutpost());
    } else if (autoChooser.getSelected().equals("finalPath()")) {
      CommandScheduler.getInstance().schedule(grabBalls());
    } else {
      System.out.println("something is very wrong if you see this");
    }
  }

  /**
   * Returns the auto chooser.
   *
   * @return The SendableChooser object for selecting autonomous commands.
   */
  public SendableChooser<String> getAutoChooser() {
    return autoChooser;
  }

  /**
   * Aligns the robot to a specified goal pose.
   *
   * @param goal The target pose to align to.
   * @return A command that aligns the robot to the specified pose.
   */
  public Command align(Pose2d goal) {
    return AutoBuilder.pathfindToPose(
            goal,
            new PathConstraints(
                    TunerConstants.kMaxLinearVelocity,
                    TunerConstants.kMaxLinearAcceleration,
                    TunerConstants.kMaxAngularVelocity,
                    TunerConstants.kMaxAngularAcceleration)).andThen(new WaitCommand(0.0001));
  }

  /**
   * Aligns the robot to a specified path then follows it.
   *
   * @param goal The target path to align to.
   * @return A command that aligns the robot to the specified path and follows it.
   */
  public Command alignToPath(PathPlannerPath goal) {
    return AutoBuilder.pathfindThenFollowPath(
            goal,
            new PathConstraints(
                    TunerConstants.kMaxLinearVelocity,
                    TunerConstants.kMaxLinearAcceleration,
                    TunerConstants.kMaxAngularVelocity,
                    TunerConstants.kMaxAngularAcceleration)).andThen(new WaitCommand(0.0001));
  }

  /**
   * Follows a specified path.
   *
   * @param path The path to follow.
   * @return A command that follows the specified path.
   */
  public Command followPath(PathPlannerPath path) {
    return AutoBuilder.followPath(path);
  }

  /**
   * Sets the starting pose of the robot based on the given path.
   * The method checks the alliance color and sets the pose accordingly.
   *
   * @param path The PathPlannerPath object representing the path to set the starting pose for.
   */
  private void setStartPose(PathPlannerPath path) {
    Pose2d startPose;
    if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
      startPose = path.flipPath().getStartingHolonomicPose().orElse(path.getStartingDifferentialPose());
    } else {
      startPose = path.getStartingHolonomicPose().orElse(path.getStartingDifferentialPose());
    }
    if (Robot.isReal()) {
      CTREDrivetrain.resetPose(startPose);
    } else {
      CTREDrivetrain.resetPose(new Pose2d(startPose.getTranslation(), startPose.getRotation().plus(CTREDrivetrain.getState().Pose.getRotation())));
    }
  }

  // Commands for different paths
  private Command bareMinimum() {
    return new SequentialCommandGroup(
            new InstantCommand(() -> setStartPose(paths.get("bareMinimum"))),
            alignToPath(paths.get("bareMinimum"))
    );
  }

  private Command StartRightOutpost() {
    return new SequentialCommandGroup(
            new InstantCommand(() -> setStartPose(paths.get("StartRight-Outpost"))),
            alignToPath(paths.get("StartRight-Outpost")),
            waitSeconds(5),
            alignToPath(paths.get("outpost-shoot")),
            run(()->new Shoot(indexer, spindexer, shooterRollers))
                    .withDeadline(waitSeconds(4))
    );
  }

  private Command grabBalls() {
    return new SequentialCommandGroup(
            new InstantCommand(() -> setStartPose(paths.get("main bump to"))),
            new InstantCommand(() -> intakePivot.setPosition(IntakePivot.IntakePosition.MID.getDegrees())),
            AutoBuilder.followPath(paths.get("main bump to")),
            new InstantCommand(() -> intakePivot.setPosition(IntakePivot.IntakePosition.DOWN.getDegrees())),
            parallel(
                    run(() -> intakeRoller.setSpeedDutyCycle(.8)),
                    AutoBuilder.followPath(paths.get("main balls"))),
            new InstantCommand(() -> intakePivot.setPosition(IntakePivot.IntakePosition.MID.getDegrees())),
            AutoBuilder.followPath(paths.get("main bump back")),
            new InstantCommand(() -> intakePivot.setPosition(IntakePivot.IntakePosition.DOWN.getDegrees())),
            new RunCommand(() -> new Shoot(indexer, spindexer, shooterRollers))
                    .withDeadline(waitSeconds(5)));
  }
}
