package frc.robot.Subsystems;

import Glitch.Lib.LEDs.AbstractLEDS;
import Glitch.Lib.LEDs.GlitchColors;
import Glitch.Lib.LEDs.GlitchLEDPatterns;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;

import static edu.wpi.first.units.Units.Second;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;

import frc.lib.util.CANdleGroup;
import frc.lib.util.CustomFireAnimation;
import frc.lib.util.CustomFireAnimation.Palette;
import com.ctre.phoenix6.signals.AnimationDirectionValue;

public class LEDSubsystem extends AbstractLEDS {

  public final Section leftSide;
  public final Section pip;
  public final Section rightSide;

  private static final double TEMP_STATE_TIME_SECONDS = 0.70;
  private static final double BLINK_RATE_SECONDS = 0.2;
  private static final double ERROR_BLINK_RATE_SECONDS = 0.1;

  private static final Timer tempStateTimer = new Timer();

  private LEDState state = LEDState.STARTUP;
  private LEDState lastState = null;

  private boolean shooting = false;
  private boolean intaking = false;
  private double currentFlywheelVelocity = 0.0;
  private double targetFlywheelVelocity = 0.0;
  private int measurementCount = 0;

  private final LEDPattern lowConfidence =
      GlitchLEDPatterns.ripple(LEDPattern.solid(Color.kYellow), 10, 20);
  private final LEDPattern reasonableConfidence =
      GlitchLEDPatterns.ripple(GlitchLEDPatterns.ace, 10, 20);
  private final LEDPattern highConfidence =
      GlitchLEDPatterns.ripple(GlitchLEDPatterns.funGradient, 20, 30);

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

  private enum LEDState {
    STARTUP(new LEDConfig(
        GlitchLEDPatterns.fire(LEDPattern.solid(GlitchColors.purple), Color.kBlack),
        LEDPattern.solid(GlitchColors.purple))),
    DISABLED(new LEDConfig(
        GlitchLEDPatterns.fire(LEDPattern.solid(GlitchColors.purple), Color.kBlack),
        LEDPattern.solid(GlitchColors.purple))),
    AUTONOMOUS(new LEDConfig(
        GlitchLEDPatterns.fire(LEDPattern.solid(Color.kRed), Color.kBlack),
        LEDPattern.solid(Color.kOrangeRed))),
    TELEOP(new LEDConfig(GlitchLEDPatterns.ace)),
    SHOOTING(new LEDConfig(GlitchLEDPatterns.blinkyGreen)),
    INTAKING(new LEDConfig(GlitchLEDPatterns.algaePickup)),
    ERROR(new LEDConfig(LEDPattern.solid(Color.kRed), true, ERROR_BLINK_RATE_SECONDS)),
    WARNING(new LEDConfig(LEDPattern.solid(Color.kYellow), true, ERROR_BLINK_RATE_SECONDS)),
    INFO(new LEDConfig(LEDPattern.solid(Color.kWhite), true, ERROR_BLINK_RATE_SECONDS));

    public final LEDConfig config;

    LEDState(LEDConfig config) {
      this.config = config;
    }
  }

  private static class LEDConfig {
    public final LEDPattern pattern;
    public final LEDPattern pipPattern;
    public final boolean blink;
    public final double blinkRate;

    LEDConfig(LEDPattern pattern) {
      this(pattern, pattern, false, BLINK_RATE_SECONDS);
    }

    LEDConfig(LEDPattern pattern, LEDPattern pipPattern) {
      this(pattern, pipPattern, false, BLINK_RATE_SECONDS);
    }

    LEDConfig(LEDPattern pattern, boolean blink, double blinkRate) {
      this(pattern, pattern, blink, blinkRate);
    }

    LEDConfig(LEDPattern pattern, LEDPattern pipPattern, boolean blink, double blinkRate) {
      this.pattern = pattern;
      this.pipPattern = pipPattern;
      this.blink = blink;
      this.blinkRate = blinkRate;
    }
  }

  public void updateVisionConfidence(int measurementCount) {
    this.measurementCount = measurementCount;
  }

  public void start() {
    state = LEDState.DISABLED;
  }

  public void autoInit() {
    shooting = false;
    intaking = false;
    state = LEDState.AUTONOMOUS;
  }

  public void teleopInit() {
    shooting = false;
    intaking = false;
    state = LEDState.TELEOP;
  }

  public void shootPatterns(double currentFlywheelVelocity, double targetFlywheelVelocity) {
    shooting = true;
    this.currentFlywheelVelocity = currentFlywheelVelocity;
    this.targetFlywheelVelocity = targetFlywheelVelocity;
  }

  public void endCommand() {
    shooting = false;
  }

  public void intakePatterns() {
    intaking = !intaking;
  }

  public void triggerError() {
    tempStateTimer.restart();
    state = LEDState.ERROR;
  }

  public void triggerWarning() {
    tempStateTimer.restart();
    state = LEDState.WARNING;
  }

  public void triggerInfo() {
    tempStateTimer.restart();
    state = LEDState.INFO;
  }

  private LEDPattern confidencePattern() {
    if (measurementCount > 2) {
      return highConfidence;
    }
    if (measurementCount == 2) {
      return reasonableConfidence;
    }
    return lowConfidence;
  }

  @Override
  public void periodic() {
    super.periodic();

    if (DriverStation.isDisabled()) {
      if (state != LEDState.STARTUP && state != LEDState.DISABLED) {
        state = LEDState.DISABLED;
      }
    } else {
      if (tempStateTimer.isRunning() && tempStateTimer.hasElapsed(TEMP_STATE_TIME_SECONDS)) {
        tempStateTimer.stop();
      }
      if (!tempStateTimer.isRunning()) {
        if (shooting) {
          state = LEDState.SHOOTING;
        } else if (intaking) {
          state = LEDState.INTAKING;
        } else {
          state = DriverStation.isAutonomous() ? LEDState.AUTONOMOUS : LEDState.TELEOP;
        }
      }
    }

    applyState();
  }

  private void applyState() {
    if (state != lastState) {
      LEDConfig cfg = state.config;
      LEDPattern side = cfg.pattern;
      LEDPattern middle = cfg.pipPattern;
      if (cfg.blink) {
        side = side.blink(Second.of(cfg.blinkRate));
        middle = middle.blink(Second.of(cfg.blinkRate));
      }
      if (state == LEDState.TELEOP || state == LEDState.AUTONOMOUS) {
        side = confidencePattern();
        middle = confidencePattern();
      }
      leftSide.setBase(side);
      leftSide.setPattern(side);
      pip.setBase(middle);
      pip.setPattern(middle);
      rightSide.setBase(side);
      rightSide.setPattern(side);
    }

    if (state == LEDState.TELEOP || state == LEDState.AUTONOMOUS) {
      LEDPattern confidence = confidencePattern();
      leftSide.setPattern(confidence);
      pip.setPattern(confidence);
      rightSide.setPattern(confidence);
    } else if (state == LEDState.SHOOTING) {
      double target = Math.max(targetFlywheelVelocity, 1.0e-9);
      double progress = Math.min(1.0, currentFlywheelVelocity / target);
      LEDPattern shootingPattern;
      if (progress >= 0.95) {
        shootingPattern = GlitchLEDPatterns.blinkyGreen;
      } else {
        shootingPattern = GlitchLEDPatterns.linearProgress(
            GlitchLEDPatterns.elevatorProgress, currentFlywheelVelocity, target);
      }
      leftSide.setPattern(shootingPattern);
      pip.setPattern(shootingPattern);
      rightSide.setPattern(shootingPattern);
    }

    lastState = state;
  }
}
