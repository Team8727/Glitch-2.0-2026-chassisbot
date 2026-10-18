package frc.robot.controls;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.leds.LED;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Shooter.ShooterControlMode;

import static edu.wpi.first.wpilibj2.command.Commands.run;

/**
 * Driver controller button bindings.
 *
 * <p>Single Xbox controller (port 0). All gameplay bindings are defined here.
 *
 * <h3>Button Map</h3>
 * <ul>
 *   <li>Left Stick — Translation (forward/strafe)</li>
 *   <li>Right Stick — Rotation</li>
 *   <li>A — Auto-aim at HUB (FieldCentricFacingAngle)</li>
 *   <li>B — Oscillation/wiggle mode</li>
 *   <li>Start — Reset field-centric heading</li>
 *   <li>Left Trigger — Intake (while held)</li>
 *   <li>Right Trigger — Shoot (adaptive, distance-based)</li>
 *   <li>Y — Indexer forward</li>
 *   <li>X — Indexer reverse (unjam)</li>
 *   <li>D-Pad Down — Manual intake (test)</li>
 *   <li>D-Pad Right — Manual shooter spin (test)</li>
 * </ul>
 */
public class Controls {
  private final CommandXboxController controller = new CommandXboxController(0);
  private final Drivetrain drivetrain;
  private final Shooter shooter;
  private final Intake intake;
  private final Indexer indexer;

  public Controls(
      Drivetrain drivetrain, Shooter shooter, Intake intake, Indexer indexer) {
    this.drivetrain = drivetrain;
    this.shooter = shooter;
    this.intake = intake;
    this.indexer = indexer;

    configureBindings();
  }

  private void configureBindings() {
    // ---- Default drive ----
    drivetrain.setDefaultCommand(DriveCommands.teleopDrive(drivetrain, controller));

    // ---- Auto-aim (A button) ----
    controller.a().whileTrue(DriveCommands.autoAimDrive(drivetrain, controller));

    // ---- Oscillation / wiggle (B button, toggle) ----
    controller.b().toggleOnTrue(DriveCommands.oscillateDrive(drivetrain, controller));

    // ---- Reset heading (Start button) ----
    controller.start().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

    // ---- Intake (Left Trigger) ----
    controller.leftTrigger().whileTrue(new IntakeCommand(intake, indexer));
    controller.leftTrigger().onFalse(run(() -> LED.instance.endCommand()));

    // ---- Shoot (Right Trigger) ----
    // The ShootCommand uses regression-based RPM targeting
    controller.rightTrigger().whileTrue(new ShootCommand(shooter, indexer, drivetrain));
    controller.rightTrigger()
        .onTrue(run(() -> LED.instance.shootPatterns(shooter.getFlywheelVelocity(), 45)))
        .onFalse(run(LED.instance::endCommand));

    // ---- Indexer forward (Y) ----
    controller.y().whileTrue(run(() -> indexer.setDutyCycle(0.8)));

    // ---- Indexer reverse / unjam (X) ----
    controller.x().whileTrue(run(() -> indexer.setDutyCycle(-1.0)));

    // ---- D-Pad test bindings ----
    controller.povDown().whileTrue(run(() -> intake.setDutyCycle(0.8)));

    controller.povRight().whileTrue(
        run(() -> shooter.setVelocity(45, ShooterControlMode.PID)));
    controller.povRight()
        .onTrue(run(() -> LED.instance.shootPatterns(shooter.getFlywheelVelocity(), 45)))
        .onFalse(run(LED.instance::endCommand));

    // ---- Auto-brake when sticks are within deadband ----
    final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    new Trigger(() ->
        Math.abs(controller.getLeftY()) < ControlsConstants.kAutoBrakeDeadband
            && Math.abs(controller.getLeftX()) < ControlsConstants.kAutoBrakeDeadband
            && Math.abs(controller.getRightX()) < ControlsConstants.kAutoBrakeDeadband
            && controller.a().negate().getAsBoolean()
    ).whileTrue(drivetrain.applyRequest(() -> brake));
  }
}