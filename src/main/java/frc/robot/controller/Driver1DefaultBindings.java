package frc.robot.controller;

import Glitch.Lib.Controller.ControllerBindings;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Autos;
import frc.robot.Commands.PointIndexAndShootCmd;
import frc.robot.Commands.RaiseIntakeCmd;
import frc.robot.Commands.Shoot;
import frc.robot.Drivetrain.CTRESwerveDrivetrain;
import frc.robot.Subsystems.*;

import static edu.wpi.first.wpilibj2.command.Commands.parallel;
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
  public final ShooterPivot shooterPivot;
  public final ShooterRollers shooterRollers;

  public Driver1DefaultBindings(
          CommandXboxController controller,
          Autos autos,
          CTRESwerveDrivetrain drivetrain,
          Spindexer spindexer,
          IntakePivot intakePivot,
          ShooterPivot shooterPivot,
          IntakeRoller intakeRoller,
          Indexer indexer,
          ShooterRollers shooterRollers
      ) {
    this.autos = autos;
    this.drivetrain = drivetrain;
    this.spindexer = spindexer;
    this.intakePivot = intakePivot;
    this.shooterPivot = shooterPivot;
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
    controller.povLeft().whileTrue(new Shoot(indexer, spindexer, shooterRollers));
    controller.leftTrigger().whileTrue(
            run(() -> indexer.setSpeedDutyCycle(-1)).alongWith(
            run(() -> spindexer.setSpeedDutyCycle(-.5))));

    controller.rightTrigger().whileTrue(run(() -> shooterRollers.setSpeedDutyCycle(.5)));
    controller.povRight().whileTrue(run(() -> indexer.setSpeedDutyCycle(.5)));
    controller.povRight().toggleOnTrue(run(() -> spindexer.setSpeedDutyCycle(.3)));
    controller.y().onTrue(new InstantCommand(() -> intakePivot.setPosition(130)));
    controller.a().onTrue(new InstantCommand(shooterPivot::zeroEncoder));
    controller.povUp().onTrue(new InstantCommand(() -> shooterPivot.setPosition(0)));
    controller.povDown().onTrue(new InstantCommand(() -> shooterPivot.setPosition(500)));


//    controller.leftBumper().onTrue(new InstantCommand(() -> shooterPivot.setPosition(100)));
//    controller.rightBumper().onTrue(new InstantCommand(() -> shooterPivot.setPosition(300)));
//    controller.b().onTrue(new InstantCommand(() -> shooterPivot.setPosition(400)));


//    controller.x().whileTrue(new PointIndexAndShootCmd(indexer, shooterPivot, shooterRoller, drivetrain, controller));
//    controller.b().whileTrue(new RaiseIntakeCmd(intakeRoller, intakePivot));
//
//    new Trigger(() -> intakePivot.getPosition() == IntakePivot.IntakePosition.DOWN.getDegrees() && controller.a().getAsBoolean())
//            .onTrue(run(() -> intakeRoller.setSpeedDutyCycle(1)));
  }
}
