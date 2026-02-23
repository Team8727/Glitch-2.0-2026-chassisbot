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
//  private final IntakePivot intakePivot = new IntakePivot();
//  private final IntakeRoller intakeRoller = new IntakeRoller();
//  public final Indexer indexer = new Indexer();
//  public final ShooterPivot shooterPivot = new ShooterPivot();
//  public final ShooterRoller shooterRoller = new ShooterRoller();

  public Driver1DefaultBindings(
          CommandXboxController controller,
          Autos autos,
          CTRESwerveDrivetrain drivetrain
//          IntakePivot intakePivot,
//          IntakeRoller intakeRoller,
//          Indexer indexer,
//          ShooterPivot shooterPivot,
//          ShooterRoller shooterRoller
      ) {
    this.autos = autos;
    // this.intakePivot = intakePivot;
    // this.intakeRoller = intakeRoller;
    // this.indexer = indexer;
    // this.shooterPivot = shooterPivot;
    // this.shooterRoller = shooterRoller;

    new CTReSwerveControls(drivetrain, controller);

    bind(controller);
  }

  @Override
  public void bind(CommandXboxController controller) {
    // controller.a().whileTrue(new PointIndexAndShoot(shooterPivot, shooterRoller, indexer, drivetrain, controller));
    // controller.povUp().whileTrue(intakePivot.setPositionCommand(IntakePivot.IntakePosition.UP.getDegrees()));
    // Put binds here
  }
}
