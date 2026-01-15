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
import frc.robot.Drivetrain.CTRESwerveDrivetrain;
import frc.robot.Drivetrain.TunerConstants;
import org.json.simple.parser.ParseException;

import java.io.IOException;
import java.util.LinkedHashMap;

public class Autos extends SubsystemBase {
//  private final LEDSubsystem m_ledSubsystem = ;
  private final LinkedHashMap<String, PathPlannerPath> paths = new LinkedHashMap<>();
  private final CTRESwerveDrivetrain CTRDrivetrain;
  private final SendableChooser<String> autoChooser = new SendableChooser<>();
  private final NetworkTableLogger logger = new NetworkTableLogger(this.getName());

  private static final Translation2d fieldCenter = new Translation2d(8.770, 4.026); // meters

  /** Creates a new Autos. */
  public Autos(CTRESwerveDrivetrain CTREdrivetrain) {
    this.CTRDrivetrain = CTREdrivetrain;

    loadPaths();
   }

  /**
   * Loads the paths from the specified path files.
   * Example:
   * <pre>
   *   loadPath("Path-Name");
   * </pre>
   */
  private void loadPaths() {
    loadPath("bareMinimum");
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
   *   autoChooser.addOption("Path-Name", "Path_Function()");
   * </pre>
   */
  public void setupAutoChooser() {
    autoChooser.setDefaultOption("BareMinimum", "bareMinimum()");
  }

  /**
   * runs the autonomous command based on the selected option in the auto chooser.
   * <pre>
   *   autoChooser.setDefaultOption("Path-Name", "Path_Function()");
   *   autoChooser.addOption("Path-Name", "Path_Function()");
   * </pre>
   */
  public void selectAuto() {
    if (autoChooser.getSelected().equals("bareMinimum()")) {
      CommandScheduler.getInstance().schedule(bareMinimum());
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
    if (DriverStation.getAlliance().orElse(Alliance.Blue) ==  Alliance.Red) {
      startPose = path.flipPath().getStartingHolonomicPose().orElse(path.getStartingDifferentialPose());
    } else {
      startPose = path.getStartingHolonomicPose().orElse(path.getStartingDifferentialPose());
    }
    if (Robot.isReal()) {
      CTRDrivetrain.resetPose(startPose);
    } else {
      CTRDrivetrain.resetPose(new Pose2d(startPose.getTranslation(), startPose.getRotation().plus(CTRDrivetrain.getState().Pose.getRotation())));
    }
  }

  // Commands for different paths
  private Command bareMinimum() {
    return new SequentialCommandGroup(
      new InstantCommand(() -> setStartPose(paths.get("bareMinimum"))),
      alignToPath(paths.get("bareMinimum"))
    );
  }
}
