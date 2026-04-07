package frc.robot.controller;

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
  public final LEDSubsystem leds;

  public Driver1DefaultBindings(
          Autos autos,
          CTRESwerveDrivetrain drivetrain,
          IntakeRoller intakeRoller,
          Indexer indexer,
          ShooterRoller shooterRoller,
          LEDSubsystem leds
      ) {
    super(0);

    this.autos = autos;
    this.drivetrain = drivetrain;
    this.intakeRoller = intakeRoller;
    this.indexer = indexer;
    this.shooterRoller = shooterRoller;
    this.leds = leds;

    configureBindings();
  }

  @Override
  protected void configureBindings() {
    new CTReSwerveControls(drivetrain, controller);

    // Put binds here
    controller.leftTrigger().toggleOnTrue(run(() -> intakeRoller.setSpeedDutyCycle(.8)));
    controller.leftTrigger().toggleOnTrue(run(() -> leds.intakePatterns()));
    //controller.rightTrigger().whileTrue(new ShootCommand(indexer, shooterRoller, 0, ShootCommand.ControlMode.PID));

    // test systems
    controller.povDown().whileTrue(run(() -> intakeRoller.setSpeedDutyCycle(.8)));
    controller.x().whileTrue(run(() -> indexer.setSpeedDutyCycle(1))); // Backwards
    controller.y().whileTrue(run(() -> indexer.setSpeedDutyCycle(-1))); // Forwards
    controller.povRight().whileTrue(run(() -> shooterRoller.setSpeedVelocity(45))); // Was 40, 45 is where drum rattling starts

//    controller.povRight().whileTrue(shooterRoller.sysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward)); // 4
//    controller.povUp().whileTrue(shooterRoller.sysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse)); // 3
//    controller.povDown().whileTrue(shooterRoller.sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward)); // 2
//    controller.povLeft().whileTrue(shooterRoller.sysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse)); // 1
  }
}
