package frc.robot.controller;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Drivetrain.CTRESwerveDrivetrain;

import java.awt.geom.Rectangle2D;

public class ZoneController extends SubsystemBase {

  private final CTRESwerveDrivetrain drivetrain;
  private final Rectangle2D zone;
  private boolean isInside;
  private boolean isEnabled = true;
  private boolean prevCommandReady = false;
  private final Command commandToManage;

  public ZoneController(CTRESwerveDrivetrain drivetrain, Rectangle2D zone, Command command) {
    this.drivetrain = drivetrain;
    this.zone = zone;
    this.commandToManage = command;
  }

  @Override
  public void periodic() {
    Pose2d currentPose = drivetrain.getState().Pose;
    isInside = zone.contains(currentPose.getX(), currentPose.getY());

    boolean commandReady = CommandReady();
    if (commandReady && !prevCommandReady) {
      CommandScheduler.getInstance().schedule(commandToManage);
    } else if (!commandReady && prevCommandReady) {
      commandToManage.cancel();
    }
    prevCommandReady = commandReady;
  }

  /**
   * Sets whether the zone controller should trigger commands.
   * @param enabled true to enable, false to disable.
   */
  public void setEnabled(boolean enabled) {
    this.isEnabled = enabled;
  }

  /**
   * Checks if the zone controller is currently enabled.
   * @return true if enabled.
   */
  public boolean isEnabled() {
    return isEnabled;
  }

  /**
   * Checks if the robot is physically inside the zone, regardless of enabled state.
   * @return true if inside.
   */
  public boolean isInsideZone() {
    return isInside;
  }

  /**
   * Checks if the robot is inside the zone AND the controller is enabled.
   * @return true if inside and enabled.
   */
  public boolean CommandReady() {
    return isInside && isEnabled;
  }
}
