// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import Glitch.Lib.NetworkTableLogger;
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
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Drivetrain.CTRESwerveDrivetrain;
import frc.robot.Drivetrain.TunerConstants;
import frc.robot.Subsystems.*;
import org.json.simple.parser.ParseException;

import java.io.IOException;
import java.util.LinkedHashMap;
import java.util.List;

import static com.pathplanner.lib.auto.AutoBuilder.followPath;
import static edu.wpi.first.wpilibj2.command.Commands.*;

public class Autos extends SubsystemBase {
  //  private final LEDSubsystem m_ledSubsystem = ;
  private final LinkedHashMap<String, PathPlannerPath> paths = new LinkedHashMap<>();
  private final CTRESwerveDrivetrain CTREDrivetrain;
  private final SendableChooser<Command> autoChooser = new SendableChooser<>();
  private final NetworkTableLogger logger = new NetworkTableLogger(this.getName());
  private final Indexer indexer;
  private final ShooterRoller shooterRoller;
  private final Spindexer spindexer;
  private final IntakePivot intakePivot;
  private final IntakeRoller intakeRoller;

  /**
   * Creates a new Autos.
   */
  public Autos(CTRESwerveDrivetrain CTREDrivetrain, Indexer indexer, ShooterRoller shooterRoller, Spindexer spindexer, IntakePivot intakePivot, IntakeRoller intakeRoller) {
    this.CTREDrivetrain = CTREDrivetrain;
    this.indexer = indexer;
    this.shooterRoller = shooterRoller;
    this.spindexer = spindexer;
    this.intakePivot = intakePivot;
    this.intakeRoller = intakeRoller;

    registerNamedCommands();
    loadPaths();
    setupAutoChooser();

    SmartDashboard.putData("Auto choices", autoChooser);
  }

  private void registerNamedCommands() {
    NamedCommands.registerCommand("intakeUp", runOnce(() -> intakePivot.setPosition(IntakePivot.IntakePosition.MID.getDegrees())));
    NamedCommands.registerCommand("intakeDown", runOnce(() -> intakePivot.setPosition(IntakePivot.IntakePosition.DOWN.getDegrees())));
    NamedCommands.registerCommand("spinRollers", intakeRoller.run(() -> intakeRoller.setSpeedDutyCycle(.5))
            .finallyDo(() -> intakeRoller.setSpeedDutyCycle(0)));
    NamedCommands.registerCommand("shoot",
            runOnce(() -> CommandScheduler.getInstance().schedule(parallel(
                    shooterRoller.run(() -> shooterRoller.setSpeedVelocity(Robot.firing.power * Math.PI * Robot.SHOOTER_LOSS_COMPENSATION))),
                    sequence(
                            waitSeconds(1.5),
                            parallel(
                                    indexer.run(() -> indexer.setSpeedDutyCycle(1)),
                                    spindexer.run(() -> spindexer.setSpeedDutyCycle(.5))
                            )
                    )
            )

    ));
  }

  /**
   * Loads the paths from the specified path files.
   */
  private void loadPaths() {
    List.of(
      "StartRight-Outpost",
      "Final plan 4.1",
      "main bump to",
      "main bump back",
      "main balls",
      "test"
    ).forEach(this::loadPath);
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
    autoChooser.setDefaultOption("bare minimum", bareMinimum());
    autoChooser.addOption("start right outpost", followPathFromStartPose(paths.get("Final plan 4.1")));
    autoChooser.addOption("grab balls", grabBalls());
    autoChooser.addOption("test", followPathFromStartPose(paths.get("test")));

    // You can also use PathPlanner's built-in auto chooser if you have .auto files
    // autoChooser = AutoBuilder.buildAutoChooser();
  }

  /**
   * @return The selected autonomous command
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public Command followPathFromStartPose(PathPlannerPath path) {
    return sequence(
            runOnce(() -> setStartPose(path)),
            AutoBuilder.followPath(path)
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
   * Sets the starting pose of the robot based on the given path.
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
    return sequence(
            runOnce(() -> setStartPose(paths.get("main bump to"))),
            parallel(
                    shooterRoller.run(() -> shooterRoller.setSpeedVelocity(Robot.firing.power * 5.6))
                            .finallyDo(() -> shooterRoller.setSpeedVelocity(0)),
                    sequence(
                            waitSeconds(1.5),
                            parallel(
                                    indexer.run(() -> indexer.setSpeedDutyCycle(1))
                                            .finallyDo(() -> indexer.setSpeedDutyCycle(0)),
                                    spindexer.run(() -> spindexer.setSpeedDutyCycle(.5))
                                            .finallyDo(() -> spindexer.setSpeedDutyCycle(0))
                            )
                    )
            )
    );
  }

  private Command grabBalls() {
    return sequence(
            runOnce(() -> setStartPose(paths.get("main bump to"))),
            intakePivot.runOnce(() -> intakePivot.setPosition(IntakePivot.IntakePosition.MID.getDegrees())),
            followPath(paths.get("main bump to")),

            intakePivot.runOnce(() -> intakePivot.setPosition(IntakePivot.IntakePosition.DOWN.getDegrees())),
            parallel(
                    sequence(
                            deadline(
                                    waitSeconds(3),
                                    intakeRoller.run(() -> intakeRoller.setSpeedDutyCycle(.8))
                            ),
                            deadline(
                                    waitSeconds(.5),
                                    intakeRoller.run(() -> intakeRoller.setSpeedDutyCycle(0))
                            )
                    ),
                    followPath(paths.get("main balls")).withDeadline(waitSeconds(3))
            ),
            intakePivot.runOnce(() -> intakePivot.setPosition(IntakePivot.IntakePosition.MID.getDegrees())),

            followPath(paths.get("main bump back")),
            intakePivot.runOnce(() -> intakePivot.setPosition(IntakePivot.IntakePosition.DOWN.getDegrees())),
            parallel(
                    shooterRoller.run(() -> shooterRoller.setSpeedVelocity(Robot.firing.power * 6.6)),
                    sequence(
                            waitSeconds(1.5),
                            parallel(
                                    indexer.run(() -> indexer.setSpeedDutyCycle(1)),
                                    spindexer.run(() -> spindexer.setSpeedDutyCycle(.5))
                            )
                    )
            )
    );
  }
}
