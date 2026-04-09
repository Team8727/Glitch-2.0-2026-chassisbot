// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Commands.ShootCommand;
import frc.robot.Drivetrain.CTRESwerveDrivetrain;
import frc.robot.Drivetrain.TunerConstants;
import frc.robot.Subsystems.Indexer;
import frc.robot.Subsystems.IntakeRoller;
import frc.robot.Subsystems.LEDSubsystem;
import frc.robot.Subsystems.ShooterRoller;
import org.json.simple.JSONArray;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;

import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;

import static edu.wpi.first.wpilibj2.command.Commands.*;

/**
 * The Autos class handles the selection and execution of autonomous routines.
 * It manages PathPlanner autos, registers named commands for use within paths,
 * and maintains a chooser for selecting autonomous sequences from the dashboard.
 */
public class Autos {
  private final LinkedHashMap<String, Command> autos = new LinkedHashMap<>();
  private final LinkedHashMap<String, PathPlannerPath> paths = new LinkedHashMap<>();
  private final CTRESwerveDrivetrain CTREDrivetrain;
  private final SendableChooser<Command> autoChooser = new SendableChooser<>();
  private final Indexer indexer;
  private final ShooterRoller shooterRoller;
  private final IntakeRoller intakeRoller;
  private final LEDSubsystem leds;

  /**
   * The list of autonomous path names to load and add to the auto chooser.
   * The first item in this list is set as the default option.
   */
  private static final List<String> AUTO_NAMES = List.of(
          "New Auto",
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
   * @param intakeRoller   The intake roller subsystem for picking up game pieces.
   * @param leds            The LED subsystem for visual feedback.
   */
  public Autos(CTRESwerveDrivetrain CTREDrivetrain, Indexer indexer, ShooterRoller shooterRoller, IntakeRoller intakeRoller, LEDSubsystem leds) {
    this.CTREDrivetrain = CTREDrivetrain;
    this.indexer = indexer;
    this.shooterRoller = shooterRoller;
    this.intakeRoller = intakeRoller;
    this.leds = leds;

    // register commands BEFORE paths
    registerNamedCommands();
    loadAutos();
    setupAutoChooser();

    SmartDashboard.putData("Auto choices", autoChooser);
  }

  /**
   * Registers named commands for use within PathPlanner paths.
   * These commands can be called by name from the PathPlanner GUI.
   */
  private void registerNamedCommands() {
    NamedCommands.registerCommand("spinRollers", intakeRoller.run(() -> intakeRoller.setDutyCycle(.5))
            .finallyDo(() -> intakeRoller.setDutyCycle(0)));
    NamedCommands.registerCommand("shoot", new ShootCommand(indexer, shooterRoller, 0, ShootCommand.ControlMode.PID));
  }

  /**
   * Loads all autonomous autos and paths from the deploy directory into the respective maps.
   */
  private void loadAutos() {
    AUTO_NAMES.forEach(this::loadRoutine);
  }

  /**
   * Loads a specific autonomous routine from the given name.
   *
   * @param name The name of the routine file (auto or path) to load.
   */
  private void loadRoutine(String name) {
    // try as auto
    try {
      autos.put(name, AutoBuilder.buildAuto(name));
    } catch (Exception e) {
      // try as path
      try {
        paths.put(name, PathPlannerPath.fromPathFile(name));
      } catch (Exception ex) {
        System.out.println("Could not load routine: " + name);
      }
    }
  }

  /**
   * Sets up the autonomous command chooser for the SmartDashboard.
   * This populates the chooser with various autonomous auto options and mirrored routines.
   */
  public void setupAutoChooser() {
    AUTO_NAMES.forEach(name -> {
      Command auto = autos.get(name);
      if (auto != null) {
        autoChooser.addOption(name, auto);
        autoChooser.addOption(name + " Mirrored", buildAuto(name, true));
      } else {
        PathPlannerPath path = paths.get(name);
        if (path == null) return;

        Command nonMirrored = followPathFromStartPose(path, false);
        Command mirrored = followPathFromStartPose(path, true);

        autoChooser.addOption(name, nonMirrored);
        autoChooser.addOption(name + " Mirrored", mirrored);
      }
    });
  }

  /**
   * Builds an autonomous command from an .auto file, with an option to mirror all paths.
   *
   * @param name   The name of the auto file to load.
   * @param mirror Whether to mirror all paths within the auto.
   * @return A {@link Command} representing the autonomous sequence.
   */
  private Command buildAuto(String name, boolean mirror) {
    try {
      String filePath = Filesystem.getDeployDirectory().getPath() + "/pathplanner/autos/" + name + ".auto";
      String content = Files.readString(Path.of(filePath));
      JSONObject json = (JSONObject) new JSONParser().parse(content);
      JSONObject commandData = (JSONObject) json.get("command");
      Object resetOdomObj = json.get("resetOdom");
      boolean resetOdom = resetOdomObj != null && (boolean) resetOdomObj;

      Command autoCommand = parseCommand(commandData, mirror);

      if (resetOdom) {
        PathPlannerPath firstPath = findFirstPath(commandData);
        if (firstPath != null) {
          PathPlannerPath pathForPose = mirror ? firstPath.mirrorPath() : firstPath;
          return sequence(
                  runOnce(() -> setStartPose(pathForPose)),
                  autoCommand
          );
        }
      }
      return autoCommand;
    } catch (Exception e) {
      System.out.println("Error building auto " + name + " (mirror=" + mirror + "): " + e.getMessage());
      return none();
    }
  }

  /**
   * Recursively parses a command JSON object from a PathPlanner .auto file.
   *
   * @param commandJson The JSON object representing the command.
   * @param mirror      Whether to mirror any paths found within the command.
   * @return The corresponding {@link Command}.
   */
  private Command parseCommand(JSONObject commandJson, boolean mirror) {
    String type = (String) commandJson.get("type");
    JSONObject data = (JSONObject) commandJson.get("data");

    switch (type) {
      case "path":
        String pathName = (String) data.get("pathName");
        PathPlannerPath path = safeLoadPath(pathName);
        if (path == null) return none();
        return AutoBuilder.followPath(mirror ? path.mirrorPath() : path);
      case "named":
        String name = (String) data.get("name");
        return NamedCommands.getCommand(name);
      case "wait":
        double waitTime = ((Number) data.get("waitTime")).doubleValue();
        return waitSeconds(waitTime);
      case "sequential":
        return sequence(parseCommandList((JSONArray) data.get("commands"), mirror));
      case "parallel":
        return parallel(parseCommandList((JSONArray) data.get("commands"), mirror));
      case "race":
        return race(parseCommandList((JSONArray) data.get("commands"), mirror));
      case "deadline":
        JSONArray commands = (JSONArray) data.get("commands");
        Command deadline = parseCommand((JSONObject) commands.get(0), mirror);
        List<Command> otherCommands = new ArrayList<>();
        for (int i = 1; i < commands.size(); i++) {
          otherCommands.add(parseCommand((JSONObject) commands.get(i), mirror));
        }
        return deadline(deadline, otherCommands.toArray(new Command[0]));
      default:
        return none();
    }
  }

  /**
   * Parses a list of command JSON objects.
   *
   * @param commandsJson The JSON array of commands.
   * @param mirror       Whether to mirror paths.
   * @return A list of {@link Command}s.
   */
  private Command[] parseCommandList(JSONArray commandsJson, boolean mirror) {
    List<Command> commands = new ArrayList<>();
    for (Object cmdObj : commandsJson) {
      commands.add(parseCommand((JSONObject) cmdObj, mirror));
    }
    return commands.toArray(new Command[0]);
  }

  /**
   * Finds the first path in a command JSON structure.
   *
   * @param commandJson The command JSON object.
   * @return The first {@link PathPlannerPath} found, or null if none.
   */
  private PathPlannerPath findFirstPath(JSONObject commandJson) {
    String type = (String) commandJson.get("type");
    JSONObject data = (JSONObject) commandJson.get("data");

    if ("path".equals(type)) {
      return safeLoadPath((String) data.get("pathName"));
    } else if (data.containsKey("commands")) {
      JSONArray commands = (JSONArray) data.get("commands");
      for (Object cmdObj : commands) {
        PathPlannerPath path = findFirstPath((JSONObject) cmdObj);
        if (path != null) return path;
      }
    }
    return null;
  }

  /**
   * Safely loads a PathPlanner path from its file name.
   *
   * @param pathName The name of the path file.
   * @return The loaded {@link PathPlannerPath}, or null if loading failed.
   */
  private PathPlannerPath safeLoadPath(String pathName) {
    try {
      return PathPlannerPath.fromPathFile(pathName);
    } catch (Exception e) {
      System.out.println("Could not load path: " + pathName);
      return null;
    }
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
    if (Robot.isRedAlliance()) {
      startPose = path.flipPath().getStartingHolonomicPose().orElse(path.getStartingDifferentialPose());
    } else {
      startPose = path.getStartingHolonomicPose().orElse(path.getStartingDifferentialPose());
    }

    CTREDrivetrain.resetPose(startPose);
  }
}
