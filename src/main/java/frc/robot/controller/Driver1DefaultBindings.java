package frc.robot.controller;

import Glitch.Lib.Controller.ControllerBindings;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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
  private final Spindexer spindexer;
  private final IntakePivot intakePivot;
  private final IntakeRoller intakeRoller;
  public final Indexer indexer;
//  public final ShooterPivot shooterPivot;
  public final ShooterRollers shooterRollers;

  public Driver1DefaultBindings(
          CommandXboxController controller,
          Autos autos,
          CTRESwerveDrivetrain drivetrain,
          Spindexer spindexer,
          IntakePivot intakePivot,
          IntakeRoller intakeRoller,
          Indexer indexer,
          ShooterRollers shooterRollers
      ) {
    this.autos = autos;
    this.drivetrain = drivetrain;
    this.spindexer = spindexer;
    this.intakePivot = intakePivot;
    this.intakeRoller = intakeRoller;
    this.indexer = indexer;
    this.shooterRollers = shooterRollers;

    new CTReSwerveControls(drivetrain, controller);

    bind(controller);
  }

  @Override
  public void bind(CommandXboxController controller) {
    // Put binds here
    controller.x().toggleOnTrue(run(() -> intakeRoller.setSpeedDutyCycle(.5)));
    controller.rightTrigger().whileTrue(run(() -> shooterRollers.setSpeedDutyCycle(.5)));
    controller.povUp().whileTrue(run(() -> indexer.setSpeedDutyCycle(.5)));
    controller.povRight().whileTrue(run(() -> spindexer.setSpeedDutyCycle(.3)));
    controller.y().onTrue(run(() -> intakePivot.setPosition(ShooterPivot.MaxShooterAngles.UP.getDegrees())));
    controller.a().onTrue(run(() -> intakePivot.setPosition(ShooterPivot.MaxShooterAngles.DOWN.getDegrees())));

//    controller.x().whileTrue(new PointIndexAndShootCmd(indexer, shooterPivot, shooterRoller, drivetrain, controller));
//    controller.b().whileTrue(new RaiseIntakeCmd(intakeRoller, intakePivot));
//
//    new Trigger(() -> intakePivot.getPosition() == IntakePivot.IntakePosition.DOWN.getDegrees() && controller.a().getAsBoolean())
//            .onTrue(run(() -> intakeRoller.setSpeedDutyCycle(1)));
  }
}
