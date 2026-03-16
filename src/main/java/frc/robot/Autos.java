// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Commands.ShootCommand;
import frc.robot.Drivetrain.CTRESwerveDrivetrain;
import frc.robot.Drivetrain.TunerConstants;
import frc.robot.Subsystems.Indexer;
import frc.robot.Subsystems.IntakeRoller;
import frc.robot.Subsystems.ShooterRoller;
import frc.robot.Subsystems.Spindexer;
import org.json.simple.parser.ParseException;

import java.io.IOException;
import java.util.LinkedHashMap;
import java.util.List;

import static edu.wpi.first.wpilibj2.command.Commands.*;

/**
 * The Autos class handles the selection and execution of autonomous routines.
 * It manages PathPlanner paths, registers named commands for use within paths,
 * and maintains a chooser for selecting autonomous sequences from the dashboard.
 */
public class Autos {
  private final LinkedHashMap<String, PathPlannerPath> paths = new LinkedHashMap<>();
  private final CTRESwerveDrivetrain CTREDrivetrain;
  private final SendableChooser<Command> autoChooser = new SendableChooser<>();
  private final Indexer indexer;
  private final ShooterRoller shooterRoller;
  private final Spindexer spindexer;
  private final IntakeRoller intakeRoller;

  /**
   * The list of autonomous path names to load and add to the auto chooser.
   * The first item in this list is set as the default option.
   */
  private static final List<String> AUTO_NAMES = List.of(
          "Final plan 4.1",
          "Final plan 4.1 New",
          "Final plan 4.1 looong",
          "test",
          "Shoot In Place"
  );

  /**
   * Constructs an Autos object to manage autonomous routines.
   *
   * @param CTREDrivetrain The swerve drivetrain used for autonomous movement and path following.
   * @param indexer        The indexer subsystem for managing game piece intake to the shooter.
   * @param shooterRoller  The shooter subsystem for launching game pieces.
   * @param spindexer      The spindexer subsystem for centering game pieces.
   * @param intakeRoller   The intake roller subsystem for picking up game pieces.
   */
  public Autos(CTRESwerveDrivetrain CTREDrivetrain, Indexer indexer, ShooterRoller shooterRoller, Spindexer spindexer, IntakeRoller intakeRoller) {
    this.CTREDrivetrain = CTREDrivetrain;
    this.indexer = indexer;
    this.shooterRoller = shooterRoller;
    this.spindexer = spindexer;
    this.intakeRoller = intakeRoller;

    // register commands BEFORE paths
    registerNamedCommands();
    loadPaths();
    setupAutoChooser();

    SmartDashboard.putData("Auto choices", autoChooser);
  }

  /**
   * Registers named commands for use within PathPlanner paths.
   * These commands can be called by name from the PathPlanner GUI.
   */
  private void registerNamedCommands() {
    NamedCommands.registerCommand("spinRollers", intakeRoller.run(() -> intakeRoller.setSpeedDutyCycle(.5))
            .finallyDo(() -> intakeRoller.setSpeedDutyCycle(0)));
    NamedCommands.registerCommand("shoot", new ShootCommand(indexer, spindexer, shooterRoller));
  }

  /**
   * Loads all autonomous paths from the deploy directory into the paths map.
   */
  private void loadPaths() {
    AUTO_NAMES.forEach(this::loadPath);
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
   * Sets up the autonomous command chooser for the SmartDashboard.
   * This populates the chooser with various autonomous path options and their mirrored versions.
   */
  public void setupAutoChooser() {
    AUTO_NAMES.forEach(name -> {
      PathPlannerPath path = paths.get(name);
      if (path == null) return;

      Command nonMirrored = followPathFromStartPose(path, false);
      Command mirrored = followPathFromStartPose(path, true);

      autoChooser.addOption(name, nonMirrored);
      autoChooser.addOption(name + " Mirrored", mirrored);
    });
  }

  /**
   * Retrieves the currently selected autonomous command from the dashboard chooser.
   *
   * @return The selected autonomous {@link Command}.
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  /**
   * Follows a PathPlanner path starting from its initial pose.
   *
   * @param path   The {@link PathPlannerPath} to follow.
   * @param mirror Whether to mirror the path based on the alliance side.
   * @return A command that resets the robot's pose to the path's start and then follows the path.
   */
  public Command followPathFromStartPose(PathPlannerPath path, boolean mirror) {
    PathPlannerPath finalPath;
    if (mirror) {
      finalPath = path.mirrorPath();
    } else {
      finalPath = path;
    }
    return sequence(
            runOnce(() -> setStartPose(finalPath)),
            AutoBuilder.followPath(finalPath)
    );
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
                    TunerConstants.kMaxAngularAcceleration)).andThen(waitSeconds(0.0001));
  }

  /**
   * Performs pathfinding to the specified target pose, then follows the path.
   *
   * @param goal The target {@link Pose2d} to navigate to.
   * @return A command that pathfinds and then aligns to the goal pose.
   */
  public Command align(Pose2d goal) {
    return AutoBuilder.pathfindToPose(
            goal,
            new PathConstraints(
                    TunerConstants.kMaxLinearVelocity,
                    TunerConstants.kMaxLinearAcceleration,
                    TunerConstants.kMaxAngularVelocity,
                    TunerConstants.kMaxAngularAcceleration)).andThen(waitSeconds(0.0001));

  }

  /**
   * Resets the robot's pose to the starting position of the given path.
   * Accounts for alliance color when determining the initial pose.
   *
   * @param path The path to extract the starting pose from.
   */
  private void setStartPose(PathPlannerPath path) {
    Pose2d startPose;
    if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
      startPose = path.flipPath().getStartingHolonomicPose().orElse(path.getStartingDifferentialPose());
    } else {
      startPose = path.getStartingHolonomicPose().orElse(path.getStartingDifferentialPose());
    }
    
    CTREDrivetrain.resetPose(startPose);
  }
}
