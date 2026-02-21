package frc.robot.controller;

import Glitch.Lib.Controller.ControllerBindings;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Autos;
import frc.robot.Drivetrain.CTRESwerveDrivetrain;
import frc.robot.Subsystems.IntakePivot;

/**
 * Default teleop controller bindings for the robot.
 */
public class Driver1DefaultBindings implements ControllerBindings {
  private final Autos autos;
  //private final IntakePivot intakePivot;

  public Driver1DefaultBindings(
      CTRESwerveDrivetrain drivetrain,
      Autos autos,
      CommandXboxController controller) {
    this.autos = autos;
    //this.intakePivot = intakePivot;

    new CTReSwerveControls(drivetrain, controller);

    bind(controller);
  }

  @Override
  public void bind(CommandXboxController controller) {
    // controller.povUp().whileTrue(intakePivot.setPositionCommand(IntakePivot.IntakePosition.UP.getDegrees()));
    // Put binds here
  }
}
