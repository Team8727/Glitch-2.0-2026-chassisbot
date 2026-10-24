package frc.robot.commands;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.shooting.ShootingRegressions;
import frc.robot.shooting.Targeting;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Shooter.ShooterControlMode;

/**
 * Shoot command with at-speed gating, power budgeting, and auto-unjam.
 *
 * <ol>
 *   <li>Spins up flywheel to target RPM (computed from projectile solver)
 *   <li>Waits until at speed (dashboard-tunable tolerance)
 *   <li>Feeds balls while monitoring total amperage — if shooter+indexer
 *       exceeds the limit, indexer duty cycle is reduced (slowing feed rate
 *       so shooter amps can drop). When shooter amps drop, indexer speeds up.
 *   <li>If indexer stalls (RPM ≈ 0 for >0.25s), auto-reverses 0.25s to clear.
 *   <li>Shooter + indexer run continuously until RT is released
 * </ol>
 */
public class ShootCommand extends SequentialCommandGroup {

    public ShootCommand(Shooter shooter, Indexer indexer, Drivetrain drivetrain) {
        SmartDashboard.putNumber("Shooter RPM Tolerance", 50.0);
        SmartDashboard.putNumber("Shooter Current Limit", 120.0);
        SmartDashboard.putNumber("Shooter Slip Comp", 0.15); // tune this if note overshoots/undershoots

        addCommands(
                sequence(
                        // Velocity is set in WaitForFlywheelSpeed.initialize() with a locked distance
                        new WaitForFlywheelSpeed(shooter, drivetrain),

                        // Feed with power management + auto-unjam
                        parallel(
                                shooter.run(() -> {}),
                                new PowerManagedIndexerFeed(shooter, indexer))
                ).finallyDo(() -> {
                    shooter.setDutyCycle(0);
                    indexer.setDutyCycle(0);
                }));
    }
}

/**
 * Waits until flywheel reaches target RPM within tolerance, or times out.
 *
 * <p>Target RPM comes from the projectile solver (physics-based muzzle velocity)
 * converted through flywheel diameter and gear ratio, with slip compensation.
 * Requirements are declared so the shooter's default command doesn't override us.
 *
 * <p>The velocity command is re-issued every cycle (execute) so the SparkMax
 * built-in PID keeps tracking even if something else touches the controller.
 */
class WaitForFlywheelSpeed extends Command {
    private final Shooter shooter;
    private final Drivetrain drivetrain;
    private double startTime;
    private double lockedMotorRPS;
    private double lockedFlywheelRPM;
    private static final double kMaxWait = Constants.kShooterSpinupTime + 3.0;

    public WaitForFlywheelSpeed(Shooter shooter, Drivetrain drivetrain) {
        this.shooter = shooter; this.drivetrain = drivetrain;
        addRequirements(shooter);
    }

    @Override public void initialize() {
        startTime = Timer.getFPGATimestamp();

        double muzzleVelocity;
        if (Robot.firing.isValid) {
            // Primary: projectile solver (physics-based, accounts for target position & motion)
            muzzleVelocity = Robot.firing.power;
        } else {
            // Fallback: regression table (empirical distance→RPM, then back to muzzle velocity)
            double dist = Targeting.getHorizontalDistance(drivetrain.getState().Pose);
            double flywheelRPM = ShootingRegressions.getRPM(dist);
            double flywheelRPS = flywheelRPM / 60.0;
            muzzleVelocity = flywheelRPS * Math.PI * Constants.kShooterFlywheelDiameterMeters;
        }
        muzzleVelocity = Math.max(muzzleVelocity, 6.0); // never go below 6 m/s

        // Convert muzzle velocity (m/s) → flywheel RPS via circumference
        double flywheelRPS = muzzleVelocity / (Math.PI * Constants.kShooterFlywheelDiameterMeters);
        // Read slip compensation from dashboard (tunable at the field — decrease if overshooting)
        double slipComp = SmartDashboard.getNumber("Shooter Slip Comp", 0.15);
        if (slipComp < 0.0) slipComp = 0.0;
        if (slipComp > 0.5) slipComp = 0.5;
        double targetFlywheelRPS = flywheelRPS / (1.0 - slipComp);
        // Convert flywheel RPS → motor RPS via gear ratio (1.6×)
        lockedMotorRPS = targetFlywheelRPS * Constants.kFlywheelGearing;
        lockedFlywheelRPM = targetFlywheelRPS * 60.0;

        shooter.setVelocity(lockedMotorRPS, ShooterControlMode.FF_AND_PID);
    }

    @Override public void execute() {
        // Re-issue the velocity command every cycle so the SparkMax PID doesn't
        // get silently overridden by the default duty-cycle command.
        shooter.setVelocity(lockedMotorRPS, ShooterControlMode.FF_AND_PID);
    }

    @Override public boolean isFinished() {
        double tol = SmartDashboard.getNumber("Shooter RPM Tolerance", 50.0);
        if (tol <= 1.0) tol = 50.0;
        double actualRPM = shooter.getFlywheelVelocity() * 60.0;
        boolean atSpeed = Math.abs(actualRPM - lockedFlywheelRPM) <= tol;
        boolean timedOut = (Timer.getFPGATimestamp() - startTime) > kMaxWait;
        if (atSpeed || timedOut) {
            SmartDashboard.putNumber("Shooter RPM Target", lockedFlywheelRPM);
            SmartDashboard.putNumber("Shooter RPM Actual", actualRPM);
            return true;
        }
        return false;
    }
}

/**
 * Runs the indexer with:
 * <ul>
 *   <li><b>Power budget:</b> if shooter+indexer current > limit, indexer duty
 *       is reduced via a P-controller. When shooter current drops, indexer
 *       naturally ramps back up.
 *   <li><b>Auto-unjam:</b> if indexer is commanded to run but RPM stays near 0
 *       for >0.25s, it reverses for 0.25s to clear the jam, then resumes forward.
 * </ul>
 */
class PowerManagedIndexerFeed extends Command {
    private final Shooter shooter;
    private final Indexer indexer;

    private double currentDuty;
    private double stallTimer = -1;
    private boolean unjamming = false;
    private double unjamTimer = -1;

    private static final double STALL_TIMEOUT = 0.25;
    private static final double UNJAM_DURATION = 0.25;
    private static final double UNJAM_DUTY = -0.5;
    private static final double kP = 0.005; // P-gain for current limiting

    public PowerManagedIndexerFeed(Shooter shooter, Indexer indexer) {
        this.shooter = shooter; this.indexer = indexer;
        addRequirements(indexer);
    }

    @Override
    public void initialize() {
        currentDuty = Constants.kShooterIndexerDutyCycle;
        indexer.setDutyCycle(currentDuty);
        stallTimer = -1;
        unjamming = false;
    }

    @Override
    public void execute() {
        double limit = SmartDashboard.getNumber("Shooter Current Limit", 120.0);
        if (limit <= 1.0) limit = 120.0;

        double shooterAmps = shooter.getOutputCurrent();
        double indexerAmps = indexer.getOutputCurrent();
        double totalAmps = shooterAmps + indexerAmps;

        // Publish for dashboard monitoring
        SmartDashboard.putNumber("Shooter Amps", shooterAmps);
        SmartDashboard.putNumber("Indexer Amps", indexerAmps);
        SmartDashboard.putNumber("Total Amps", totalAmps);

        if (unjamming) {
            // In the middle of an unjam reverse pulse
            if ((Timer.getFPGATimestamp() - unjamTimer) > UNJAM_DURATION) {
                // Switch back to forward
                currentDuty = Constants.kShooterIndexerDutyCycle;
                indexer.setDutyCycle(currentDuty);
                unjamming = false;
                stallTimer = -1;
            }
            return; // Don't check for stall while unjamming
        }

        // === Power budget P-controller ===
        if (totalAmps > limit) {
            // Over current limit — reduce indexer duty
            currentDuty -= kP * (totalAmps - limit);
            if (currentDuty < 0.0) currentDuty = 0.0;
            indexer.setDutyCycle(currentDuty);
        } else if (totalAmps < limit * 0.7 && currentDuty < Constants.kShooterIndexerDutyCycle) {
            // Headroom available — ramp indexer back up
            currentDuty += kP * (limit - totalAmps);
            if (currentDuty > Constants.kShooterIndexerDutyCycle)
                currentDuty = Constants.kShooterIndexerDutyCycle;
            indexer.setDutyCycle(currentDuty);
        }

        // === Auto-unjam ===
        double indexerRPM = Math.abs(indexer.getVelocity() * 60.0);
        if (currentDuty > 0.02 && indexerRPM < 5.0) {
            // Should be moving but isn't
            if (stallTimer < 0) {
                stallTimer = Timer.getFPGATimestamp();
            } else if ((Timer.getFPGATimestamp() - stallTimer) > STALL_TIMEOUT) {
                // Stalled — reverse to clear
                currentDuty = UNJAM_DUTY;
                indexer.setDutyCycle(currentDuty);
                unjamming = true;
                unjamTimer = Timer.getFPGATimestamp();
            }
        } else {
            stallTimer = -1; // Reset stall timer — it's moving
        }
    }

    @Override public boolean isFinished() { return false; }
}