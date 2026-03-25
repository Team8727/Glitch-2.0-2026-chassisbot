package frc.robot.controller;

import Glitch.Lib.Controller.ControllerBindings;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Autos;
import frc.robot.Commands.ShootCommand;
import frc.robot.Commands.ShootCommandFF;
import frc.robot.Commands.ShootCommandStateSpace;
import frc.robot.Drivetrain.CTRESwerveDrivetrain;
import frc.robot.Subsystems.*;

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
  public final ShooterRoller shooterRoller;

  public Driver1DefaultBindings(
          CommandXboxController controller,
          Autos autos,
          CTRESwerveDrivetrain drivetrain,
          Spindexer spindexer,
          IntakePivot intakePivot,
          IntakeRoller intakeRoller,
          Indexer indexer,
          ShooterRoller shooterRoller
      ) {
    this.autos = autos;
    this.drivetrain = drivetrain;
    this.spindexer = spindexer;
    this.intakePivot = intakePivot;
    this.intakeRoller = intakeRoller;
    this.indexer = indexer;
    this.shooterRoller = shooterRoller;

    swerveControls= new CTReSwerveControls(drivetrain, controller);

    bind(controller);
  }

  @Override
  public void bind(CommandXboxController controller) {
    // Put binds here
    controller.leftTrigger().toggleOnTrue(run(() -> intakeRoller.setSpeedDutyCycle(.8)));
    controller.rightTrigger().whileTrue(new ShootCommandFF(indexer, spindexer, shooterRoller));
    //controller.leftBumper().whileTrue(new ShootCommandFF(indexer, spindexer, shooterRoller));
    controller.y().whileTrue(
            run(() -> indexer.setSpeedDutyCycle(-1)).alongWith(
            run(() -> spindexer.setSpeedDutyCycle(-.5))));

//    controller.povRight().whileTrue(shooterRoller.sysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward)); // 4
//    controller.povUp().whileTrue(shooterRoller.sysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse)); // 3
//    controller.povDown().whileTrue(shooterRoller.sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward)); // 2
//    controller.povLeft().whileTrue(shooterRoller.sysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse)); // 1

    // Testing flywheel state space control (
    //controller.rightBumper().whileTrue(new ShootCommandStateSpace(indexer, spindexer, shooterRoller));
//    controller.rightBumper().whileTrue(run(() -> {
//      shooterRoller.m_loop.setNextR(VecBuilder.fill(1.125 * Robot.firing.power * Math.PI * Robot.SHOOTER_LOSS_COMPENSATION)); // In rad/sec
//      shooterRoller.m_loop.correct(VecBuilder.fill(shooterRoller.getVelocity()));
//      shooterRoller.m_loop.predict(0.020);
//      double nextVoltage = shooterRoller.m_loop.getU(0);
//      shooterRoller.setSpeedVoltage(nextVoltage);
//    })).whileFalse(run(() -> {
//      shooterRoller.m_loop.setNextR(VecBuilder.fill(0));
//      shooterRoller.m_loop.correct(VecBuilder.fill(shooterRoller.getVelocity()));
//      shooterRoller.m_loop.predict(0.020);
//      double nextVoltage = shooterRoller.m_loop.getU(0);
//      shooterRoller.setSpeedVoltage(nextVoltage);
//    }));
//    controller.povRight().onTrue(new InstantCommand(shooterPivot::zeroEncoder));
//    controller.povUp().onTrue(new InstantCommand(() -> shooterPivot.setPosition(30)));
//    controller.povDown().onTrue(new InstantCommand(() -> shooterPivot.setPosition(50)));
//    controller.povLeft().onTrue(new InstantCommand(() -> shooterPivot.setPosition(75)));

//    controller.povRight().whileTrue(run(()->spindexer.setSpeedDutyCycle(.5)));



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
