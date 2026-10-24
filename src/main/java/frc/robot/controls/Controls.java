package frc.robot.controls;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.SmartShootCommand;
import frc.robot.shooting.Targeting;
import frc.robot.Constants;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
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
 *   <li>A — Auto-aim at selected target (hold to drive while facing target)</li>
 *   <li>B — Oscillation/wiggle mode (auto-aim with sinusoidal heading)</li>
 *   <li>Start — Reset field-centric heading</li>
 *   <li>Left Trigger — Intake (while held)</li>
 *   <li>Right Trigger — Smart shoot (auto-aim + spin up + power-managed feed)</li>
 *   <li>Y — Indexer forward</li>
 *   <li>X — Indexer reverse (unjam)</li>
 *   <li>Right Bumper — Cycle aim target (HUB → Neutral Pass → Alliance → HUB)</li>
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

  private static final java.util.HashSet<String> publishedOptions = new java.util.HashSet<>();

  public Controls(
      Drivetrain drivetrain, Shooter shooter, Intake intake, Indexer indexer) {
    this.drivetrain = drivetrain;
    this.shooter = shooter;
    this.intake = intake;
    this.indexer = indexer;

    configureBindings();
    initTunableOptions();
  }

  /** Publishes dashboard tunable option toggles (idempotent — only publishes once per option). */
  private static void tunableBoolean(String key, boolean defaultValue) {
    if (publishedOptions.add(key)) {
      SmartDashboard.putBoolean(key, defaultValue);
    }
  }

  /** Must be called after configureBindings so SmartDashboard topics exist. */
  private void initTunableOptions() {
    tunableBoolean("TunableOptions/Automatically Aim", true);
    tunableBoolean("TunableOptions/Shoot Without Vision", true);
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

    // ---- Intake + Indexer (Left Trigger) — toggles on/off, default command stops both ----
    controller.leftTrigger().toggleOnTrue(run(() -> {
        intake.setDutyCycle(Constants.kIntakeDutyCycle);
        indexer.setDutyCycle(Constants.kIndexerForwardDutyCycle);
        intake.setIntaking(true);
    }));

    // ---- Shoot (Right Trigger) ----
    // SmartShoot: auto-rotates to face the HUB while spinning up and feeding
    controller.rightTrigger().whileTrue(new SmartShootCommand(shooter, indexer, drivetrain));

    // ---- Indexer forward (Y) ----
    controller.y().whileTrue(run(() -> indexer.setDutyCycle(0.8)));

    // ---- Indexer reverse / unjam (X) ----
    controller.x().whileTrue(run(() -> indexer.setDutyCycle(-1.0)));

    // ---- Cycle aim target (Right Bumper) — HUB → Neutral Pass → Alliance —----
    controller.rightBumper().onTrue(run(Targeting::cycleTargetForward));

    // ---- D-Pad test bindings ----
    controller.povDown().whileTrue(run(() -> intake.setDutyCycle(0.8)));

    controller.povRight().whileTrue(
        run(() -> shooter.setVelocity(45, ShooterControlMode.PID)));

    // ---- Auto-brake when sticks are within deadband, unless driver is actively shooting/aiming/intaking ----
    final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    new Trigger(() ->
        Math.abs(controller.getLeftY()) < ControlsConstants.kAutoBrakeDeadband
            && Math.abs(controller.getLeftX()) < ControlsConstants.kAutoBrakeDeadband
            && Math.abs(controller.getRightX()) < ControlsConstants.kAutoBrakeDeadband
            && controller.a().negate().getAsBoolean()
            && controller.rightTrigger().negate().getAsBoolean()
            && controller.leftTrigger().negate().getAsBoolean()
    ).whileTrue(drivetrain.applyRequest(() -> brake));
  }
}