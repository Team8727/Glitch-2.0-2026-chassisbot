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
  private final CTReSwerveControls swerveControls;
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

    swerveControls= new CTReSwerveControls(drivetrain, controller);

    bind(controller);
  }

  @Override
  public void bind(CommandXboxController controller) {
    // Put binds here
    controller.leftTrigger().toggleOnTrue(run(() -> intakeRoller.setSpeedDutyCycle(.8)));
    controller.rightTrigger().whileTrue(new Shoot(indexer, spindexer, shooterRollers));
    controller.y().whileTrue(
            run(() -> indexer.setSpeedDutyCycle(-1)).alongWith(
            run(() -> spindexer.setSpeedDutyCycle(-.5))));

//    controller.a().whileTrue(new PointIndexAndShootCmd());
    controller.x().onTrue(new RaiseIntakeCmd(intakeRoller, intakePivot));
//    controller.povRight().onTrue(new InstantCommand(shooterPivot::zeroEncoder));
    controller.povUp().onTrue(new InstantCommand(() -> shooterPivot.setPosition(30)));
    controller.povDown().onTrue(new InstantCommand(() -> shooterPivot.setPosition(50)));
    controller.povLeft().onTrue(new InstantCommand(() -> shooterPivot.setPosition(75)));



//    controller.leftBumper().onTrue(new InstantCommand(() -> shooterPivot.setPosition(100)));
//    controller.rightBumper().onTrue(new InstantCommand(() -> shooterPivot.setPosition(300)));
//    controller.b().onTrue(new InstantCommand(() -> shooterPivot.setPosition(400)));


//    controller.a().whileTrue(new PointIndexAndShootCmd(indexer, shooterPivot, shooterRollers, drivetrain, controller));
//    controller.b().whileTrue(new RaiseIntakeCmd(intakeRoller, intakePivot));
//
//    new Trigger(() -> intakePivot.getPosition() == IntakePivot.IntakePosition.DOWN.getDegrees() && controller.a().getAsBoolean())
//            .onTrue(run(() -> intakeRoller.setSpeedDutyCycle(1)));
  }
}
