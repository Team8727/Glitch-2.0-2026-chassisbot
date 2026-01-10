package frc.robot.controller;

import Glitch.Lib.Controller.ControllerBindings;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Drivetrain.CTRESwerveDrivetrain;

/**
 * Default teleop controller bindings for the robot.
 */
public class Driver1DefaultBindings implements ControllerBindings {
//  private final Autos autos;

  public Driver1DefaultBindings(
      CTRESwerveDrivetrain drivetrain,
//      Autos autos,
      CommandXboxController controller) {
//    this.autos = autos;

    new CTReSwerveControls(drivetrain, controller);

    bind(controller);
  }

  @Override
  public void bind(CommandXboxController controller) {
    // Put binds here

  }
}
