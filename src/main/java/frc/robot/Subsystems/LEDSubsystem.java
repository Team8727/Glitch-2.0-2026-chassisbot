package frc.robot.Subsystems;

import static edu.wpi.first.units.Units.Second;

import Glitch.Lib.LEDs.AbstractLEDS;
import Glitch.Lib.LEDs.GlitchColors;
import Glitch.Lib.LEDs.GlitchLEDPatterns;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;

/**
 * LED state machine subsystem.
 *
 * <p>Architecture is a port of Lynk Robotics' 2026 RobotCode LED subsystem
 * (https://github.com/LynkRobotics/RobotCode2026Public), re-rendered onto Team 8727's
 * GlitchLib {@link AbstractLEDS} sections with team colors and this robot's context:
 * <ul>
 *   <li>Team purple fire for the disabled/idle baseline</li>
 *   <li>Vision-confidence patterns as the autonomous/teleop base</li>
 *   <li>Flywheel spin-up progress while shooting (see {@link #shootPatterns(double, double)})</li>
 *   <li>Intake pattern driven by {@link IntakeRoller#isIntaking}</li>
 *   <li>Endgame countdown from {@link DriverStation#getMatchTime()}</li>
 *   <li>Temporary ERROR / WARNING / INFO states with a shared timer</li>
 * </ul>
 */
public class LEDSubsystem extends AbstractLEDS {

    public final Section leftSide;
    public final Section pip;
    public final Section rightSide;

    public LEDSubsystem() {
        super(91, 40, 11, -40);

        leftSide = getSections().get(0);
        pip = getSections().get(1);
        pip.setBase(LEDPattern.kOff);
        rightSide = getSections().get(2);
    }

    static LEDSubsystem instance;

    public static LEDSubsystem getInstance() {
        if (instance == null) {
            instance = new LEDSubsystem();
        }
        return instance;
    }

    /* -=-=-=- State machine (copied from Lynk Robotics 2026 RobotCode) -=-=-=- */

    public enum LEDState {
        STARTUP,
        DISABLED,
        AUTONOMOUS,
        ACTIVE_SHOOTING,
        INACTIVE_SHOOTING,
        INTAKING,
        IDLE,
        ENDGAME,
        ERROR,
        WARNING,
        INFO
    }

    private static LEDState state = LEDState.STARTUP;
    private static LEDState lastState = null;

    private static final Timer tempStateTimer = new Timer();

    /* Populated from the driver controls. */
    private boolean shootingActive = false;
    private double currentFlywheelVelocity = 0.0;
    private double targetFlywheelVelocity = 1.0;

    /* Vision-confidence base fed by Robot.java; used for AUTO and IDLE. */
    private LEDPattern visionBase = GlitchLEDPatterns.purple;

    /* Misc timing */
    public static final double tempStateTime = 0.70;   // How long warnings/errors/info last
    public static final double blinkRate = 0.2;        // Regular blink rate
    public static final double errorBlinkRate = 0.1;   // Blink rate for errors and warnings
    public static final double endgameCountdown = 30.0;

    /** Updates the base pattern shown during AUTO and IDLE (from Robot.java's vision confidence). */
    public void setVisionBase(LEDPattern pattern) {
        if (pattern != null) {
            visionBase = pattern;
        }
    }

    /** Marks the shooter command as active and remembers the spin-up progress targets. */
    public void shootPatterns(double currentFlywheelVelocity, double targetFlywheelVelocity) {
        this.shootingActive = true;
        this.currentFlywheelVelocity = currentFlywheelVelocity;
        this.targetFlywheelVelocity = targetFlywheelVelocity;
    }

    /** Clears the shooting request, returning the strip to its idle mode. */
    public void endCommand() {
        this.shootingActive = false;
        if (tempStateTimer.isRunning()) {
            tempStateTimer.stop();
        }
    }

    public static void triggerError() {
        state = LEDState.ERROR;
        tempStateTimer.restart();
    }

    public static void triggerWarning() {
        state = LEDState.WARNING;
        tempStateTimer.restart();
    }

    public static void triggerInfo() {
        state = LEDState.INFO;
        tempStateTimer.restart();
    }

    /** Called from Robot.java's disabledInit. */
    public void start() {
        state = LEDState.DISABLED;
    }

    /** Called from Robot.java's autonomousInit. */
    public void autoInit() {
        endCommand();
        state = LEDState.AUTONOMOUS;
    }

    /** Called from Robot.java's teleopInit. */
    public void teleopInit() {
        endCommand();
        state = LEDState.IDLE;
    }

    @Override
    public void periodic() {
        double playerCountdown = 1.0;
        boolean disabledNext = false;

        /* Determine the proper LED state */
        if (DriverStation.isDisabled()) {
            if (state != LEDState.STARTUP && state != LEDState.DISABLED) {
                state = LEDState.DISABLED;
            }
        } else {
            /* Check if the temporary state should be cleared */
            if (tempStateTimer.isRunning() && tempStateTimer.hasElapsed(tempStateTime)) {
                tempStateTimer.stop();
            }

            /* Compute the proper state if it's not temporarily overridden */
            if (!tempStateTimer.isRunning()) {
                double timeLeft = DriverStation.getMatchTime();
                boolean activeShooting = shootingActive
                        && currentFlywheelVelocity / targetFlywheelVelocity >= 0.95;

                if (activeShooting) {
                    state = LEDState.ACTIVE_SHOOTING;
                } else if (shootingActive) {
                    state = LEDState.INACTIVE_SHOOTING;
                } else if (IntakeRoller.isIntaking) {
                    state = LEDState.INTAKING;
                } else if (DriverStation.isAutonomous()) {
                    state = LEDState.AUTONOMOUS;
                } else if (timeLeft > 0 && timeLeft <= endgameCountdown) {
                    /* Player-led endgame countdown */
                    playerCountdown = timeLeft / endgameCountdown;
                    disabledNext = true;
                    state = LEDState.ENDGAME;
                } else {
                    state = LEDState.IDLE;
                }
            }
        }

        /* Dashboard overrides for debugging */
        String override = SmartDashboard.getString("LED Mode Override", "");
        if (override != null && override.length() > 0) {
            try {
                state = LEDState.valueOf(override);
            } catch (IllegalArgumentException e) {
                /* Unknown override name; ignore */
            }
        }
        double overridePct = SmartDashboard.getNumber("LED Pct Override", -1.0);
        if (overridePct >= 0.0) {
            playerCountdown = overridePct;
            disabledNext = false;
        }

        /* Re-apply the pattern every cycle so scroll/blink/progress animations stay live */
        applyState(playerCountdown, disabledNext);

        if (state != lastState) {
            SmartDashboard.putString("LED/State", state.toString());
            lastState = state;
        }

        super.periodic();
    }

    /** Renders the current {@link LEDState} onto the strip and pip sections. */
    private void applyState(double playerCountdown, boolean countdownMode) {
        LEDPattern strip = visionBase;
        LEDPattern center = GlitchLEDPatterns.purple;

        switch (state) {
            case STARTUP:
            case DISABLED:
                strip = GlitchLEDPatterns.fire(LEDPattern.solid(GlitchColors.purple), Color.kBlack);
                center = GlitchLEDPatterns.purple;
                break;
            case AUTONOMOUS:
                strip = visionBase;
                center = GlitchLEDPatterns.purple;
                break;
            case IDLE:
                strip = visionBase;
                center = LEDPattern.solid(Color.kGreen);
                break;
            case ACTIVE_SHOOTING:
                strip = GlitchLEDPatterns.blinkyGreen;
                center = LEDPattern.solid(Color.kGreen);
                break;
            case INACTIVE_SHOOTING:
                strip = GlitchLEDPatterns.linearProgress(
                        GlitchLEDPatterns.elevatorProgress,
                        currentFlywheelVelocity,
                        targetFlywheelVelocity);
                center = LEDPattern.solid(Color.kYellow);
                break;
            case INTAKING:
                strip = GlitchLEDPatterns.algaePickup;
                center = GlitchLEDPatterns.purple;
                break;
            case ENDGAME:
                if (countdownMode) {
                    double fractionLeft = Math.max(0.0, Math.min(1.0, playerCountdown));
                    strip = visionBase.mask(LEDPattern.progressMaskLayer(() -> fractionLeft));
                    center = GlitchLEDPatterns.purple.blink(Second.of(errorBlinkRate));
                } else {
                    strip = GlitchLEDPatterns.purple.blink(Second.of(blinkRate));
                    center = GlitchLEDPatterns.purple;
                }
                break;
            case ERROR:
                strip = LEDPattern.solid(Color.kRed).blink(Second.of(errorBlinkRate));
                center = LEDPattern.solid(Color.kRed);
                break;
            case WARNING:
                strip = LEDPattern.solid(Color.kYellow).blink(Second.of(errorBlinkRate));
                center = LEDPattern.solid(Color.kYellow);
                break;
            case INFO:
                strip = LEDPattern.solid(Color.kWhite).blink(Second.of(errorBlinkRate));
                center = LEDPattern.solid(Color.kWhite);
                break;
            default:
                break;
        }

        leftSide.setPattern(strip);
        pip.setPattern(center);
        rightSide.setPattern(strip);
    }
}
