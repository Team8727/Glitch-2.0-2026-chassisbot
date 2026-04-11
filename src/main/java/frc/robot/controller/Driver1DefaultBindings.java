package frc.robot.controller;

import Glitch.Lib.BaseMechanisms.Roller;
import Glitch.Lib.Controller.Controller;
import frc.robot.Autos;
import frc.robot.Commands.ShootCommand;
import frc.robot.Drivetrain.CTRESwerveDrivetrain;
import frc.robot.Subsystems.Indexer;
import frc.robot.Subsystems.IntakeRoller;
import frc.robot.Subsystems.LEDSubsystem;
import frc.robot.Subsystems.ShooterRoller;

import static edu.wpi.first.wpilibj2.command.Commands.run;

/**
 * Default teleop controller bindings for the robot.
 */
public class Driver1DefaultBindings extends Controller {

  private final Autos autos;
  private final CTRESwerveDrivetrain drivetrain;
  private final IntakeRoller intakeRoller;
  public final Indexer indexer;
  public final ShooterRoller shooterRoller;
  private final LEDSubsystem leds = LEDSubsystem.getInstance();

  public Driver1DefaultBindings(
          Autos autos,
          CTRESwerveDrivetrain drivetrain,
          IntakeRoller intakeRoller,
          Indexer indexer,
          ShooterRoller shooterRoller
      ) {
    super(0);

    this.autos = autos;
    this.drivetrain = drivetrain;
    this.intakeRoller = intakeRoller;
    this.indexer = indexer;
    this.shooterRoller = shooterRoller;

    configureBindings();
  }

  @Override
  protected void configureBindings() {
    new CTReSwerveControls(drivetrain, controller);

    // Put binds here
    controller.leftTrigger().toggleOnTrue(run(() -> intakeRoller.setDutyCycle(.8)));
    controller.leftTrigger().toggleOnTrue(run(() -> leds.intakePatterns()));
    controller.leftTrigger().onTrue(run(() -> leds.endCommand()));
    // controller.leftTrigger().toggleOnTrue(new IntakeCommand(intakeRoller, 0.8));
    // controller.leftTrigger().onTrue(run(leds::intakePatterns));
    controller.rightTrigger().whileTrue(new ShootCommand(indexer, shooterRoller,0, ShootCommand.ControlMode.PID));
    controller.rightTrigger().whileTrue(run(() -> leds.shootPatterns(shooterRoller.getFlywheelVelocity(), leds.motorSpeed / (1 - .268))));
    controller.rightTrigger().onFalse(run(() -> leds.endCommand()));
    // controller.rightTrigger().onTrue(run(() -> leds.shootPatterns(shooterRoller.getFlywheelVelocity(), 45))).onFalse(run(leds::endCommand));

    // test systems
    controller.povDown().whileTrue(run(() -> intakeRoller.setDutyCycle(.8)));
    controller.y().whileTrue(run(() -> indexer.setDutyCycle(1))); // Forwards
    controller.x().whileTrue(run(() -> indexer.setDutyCycle(-1))); // Backwards
    controller.povRight().whileTrue(run(() -> shooterRoller.setVelocity(45, Roller.ControlMode.PID)));
    // controller.povRight().onTrue(run(() -> leds.shootPatterns(shooterRoller.getFlywheelVelocity(), 45))).onFalse(run(leds::endCommand)); // Was 40, 45 is where drum rattling starts

//    controller.povRight().whileTrue(shooterRoller.sysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward)); // 4
//    controller.povUp().whileTrue(shooterRoller.sysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse)); // 3
//    controller.povDown().whileTrue(shooterRoller.sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward)); // 2
//    controller.povLeft().whileTrue(shooterRoller.sysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse)); // 1
  }
}
