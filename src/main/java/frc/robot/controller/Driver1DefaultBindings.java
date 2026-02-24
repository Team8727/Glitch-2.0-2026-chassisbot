package frc.robot.controller;

import Glitch.Lib.Controller.ControllerBindings;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Autos;
import frc.robot.Commands.PointIndexAndShootCmd;
import frc.robot.Commands.RaiseIntakeCmd;
import frc.robot.Drivetrain.CTRESwerveDrivetrain;
import frc.robot.Subsystems.*;

import static edu.wpi.first.wpilibj2.command.Commands.run;

/**
 * Default teleop controller bindings for the robot.
 */
public class Driver1DefaultBindings implements ControllerBindings {
  private final Autos autos;
  private final CTRESwerveDrivetrain drivetrain;
//  private final IntakePivot intakePivot;
//  private final IntakeRoller intakeRoller;
//  public final Indexer indexer;
//  public final ShooterPivot shooterPivot;
//  public final ShooterRoller shooterRoller;

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
    this.drivetrain = drivetrain;
//    this.intakePivot = intakePivot;
//    this.intakeRoller = intakeRoller;
//    this.indexer = indexer;
//    this.shooterPivot = shooterPivot;
//    this.shooterRoller = shooterRoller;

    new CTReSwerveControls(drivetrain, controller);

    bind(controller);
  }

  @Override
  public void bind(CommandXboxController controller) {
    // Put binds here
//    controller.x().whileTrue(new PointIndexAndShootCmd(indexer, shooterPivot, shooterRoller, drivetrain, controller));
//    controller.b().whileTrue(new RaiseIntakeCmd(intakeRoller, intakePivot));
//
//    new Trigger(() -> intakePivot.getPosition() == IntakePivot.IntakePosition.DOWN.getDegrees() && controller.a().getAsBoolean())
//            .onTrue(run(() -> intakeRoller.setSpeedDutyCycle(1)));
  }
}
