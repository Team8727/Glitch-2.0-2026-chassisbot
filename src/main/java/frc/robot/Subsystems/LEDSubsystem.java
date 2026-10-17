package frc.robot.Subsystems;

import Glitch.Lib.LEDs.AbstractLEDS;
import Glitch.Lib.LEDs.GlitchColors;
import Glitch.Lib.LEDs.GlitchLEDPatterns;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;

import static edu.wpi.first.units.Units.Second;

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

  // --- CANdle backend ---
  private CANdleGroup candleGroup;
  private boolean candleActive = false;

  // --- Green-purple fire animation (disabled/idle default) ---
  private CustomFireAnimation greenPurpleFire;
  private boolean fireActive = false;

  // --- State machine (ported from Lynk/PWM version) ---
  private static final double TEMP_STATE_TIME_SECONDS = 0.70;
  private static final double BLINK_RATE_SECONDS = 0.2;
  private static final double ERROR_BLINK_RATE_SECONDS = 0.1;

  private static final Timer tempStateTimer = new Timer();
  private double blinkTimer = 0;
  private boolean blinkOff = false;

  private LEDState state = LEDState.STARTUP;
  private LEDState lastState = null;

  private boolean shooting = false;
  private boolean intaking = false;
  private double currentFlywheelVelocity = 0.0;
  private double targetFlywheelVelocity = 0.0;
  private int measurementCount = 0;
  private double loopDt = 0.02;

  /** CAN device ID — set this to match your CANdle's configured ID in Phoenix Tuner. */
  private static final int CANDLE_CAN_ID = 49;

  public LEDSubsystem() {
    // 88 total LEDs: left 40, middle pip 8, right 40 (reversed)
    super(88, 40, 8, -40);

    leftSide = getSections().get(0);
    pip = getSections().get(1);
    pip.setBase(LEDPattern.kOff);
    rightSide = getSections().get(2);

    initCandle();
  }

  static LEDSubsystem instance;

  public static LEDSubsystem getInstance() {
    if (instance == null) {
      instance = new LEDSubsystem();
    }
    return instance;
  }

  // ========== CANdle init ==========

  private void initCandle() {
    // Use the no-bus constructor so the CANdle is found on the roboRIO's native
    // CAN 2.0 port rather than on a CANivore bus. If you move the CANdle to a
    // CANivore later, add TunerConstants.kCANBus as the second argument.
    CANdle candle = new CANdle(CANDLE_CAN_ID);
    candleGroup = new CANdleGroup(candle);

    greenPurpleFire = new CustomFireAnimation(0, 87)
        .withFrameRate(60).withSparking(0.8).withCooling(0.65)
        .withBrightness(1.0).withPalette(Palette.GREEN_PURPLE)
        .withDirection(AnimationDirectionValue.Forward);

    candleActive = true;
    fireActive = true;
    greenPurpleFire.reset();
  }

  // ========== Color helpers ==========

  private void allOff() { candleGroup.setLEDs(0, 0, 0, 0, 88); }

  private void allSolid(int r, int g, int b) { candleGroup.setLEDs(r, g, b, 0, 88); }

  /** Fills a percentage of the strip from LED 0 upward. */
  private void progressBar(double pct, int r, int g, int b) {
    int lit = (int) Math.round(88 * pct);
    for (int section = 0; section < 88; section += 88) {
      int count = Math.min(lit - section, 88 - section);
      if (count > 0) candleGroup.setLEDs(r, g, b, section, count);
    }
    int unlit = 88 - lit;
    if (unlit > 0) candleGroup.setLEDs(0, 0, 0, lit, unlit);
  }

  // ========== Fire rendering helpers ==========

  /** Renders fire on left (0-39) and right (48-87) sections with a solid pip (40-47). */
  private void fireOnSides(Palette palette, int pipR, int pipG, int pipB) {
    CustomFireAnimation leftFire = new CustomFireAnimation(0, 39)
        .withFrameRate(60).withSparking(0.7).withCooling(0.5)
        .withBrightness(1.0).withPalette(palette)
        .withDirection(AnimationDirectionValue.Forward);
    candleGroup.setLEDs(pipR, pipG, pipB, 40, 8);
    leftFire.update(candleGroup);
    // right side uses the same palette but mirrored
    CustomFireAnimation rightFire = new CustomFireAnimation(48, 87)
        .withFrameRate(60).withSparking(0.7).withCooling(0.5)
        .withBrightness(1.0).withPalette(palette)
        .withDirection(AnimationDirectionValue.Forward);
    rightFire.update(candleGroup);
  }

  // ========== State machine (LEDState enum + config) ==========

  private enum LEDState {
    STARTUP, DISABLED, AUTONOMOUS, TELEOP, SHOOTING, INTAKING,
    ERROR, WARNING, INFO;

    public final boolean blink;
    public final double blinkRate;

    LEDState() { this(false, BLINK_RATE_SECONDS); }
    LEDState(boolean blink, double blinkRate) {
      this.blink = blink;
      this.blinkRate = blinkRate;
    }
  }

  // ========== Public API ==========

  public void start() {
    fireActive = true;
    state = LEDState.DISABLED;
  }

  public void autoInit() {
    shooting = false; intaking = false; fireActive = false;
    state = LEDState.AUTONOMOUS;
  }

  public void teleopInit() {
    shooting = false; intaking = false; fireActive = false;
    state = LEDState.TELEOP;
  }

  public void updateVisionConfidence(int count) { this.measurementCount = count; }

  public void shootPatterns(double currentFlywheelVelocity, double targetFlywheelVelocity) {
    fireActive = false;
    shooting = true;
    this.currentFlywheelVelocity = currentFlywheelVelocity;
    this.targetFlywheelVelocity = targetFlywheelVelocity;
  }

  public void endCommand() { shooting = false; }

  public void intakePatterns() { intaking = !intaking; fireActive = false; }

  public void triggerError() { tempStateTimer.restart(); state = LEDState.ERROR; fireActive = false; }
  public void triggerWarning() { tempStateTimer.restart(); state = LEDState.WARNING; fireActive = false; }
  public void triggerInfo() { tempStateTimer.restart(); state = LEDState.INFO; fireActive = false; }

  // ========== Main periodic ==========

  @Override
  public void periodic() {
    loopDt = 0.02; // approximate 50 Hz

    if (!candleActive) return;

    // Fire animation (disabled/startup)
    if (fireActive) {
      greenPurpleFire.update(candleGroup);
      return;
    }

    // Temp state (error/warning/info) timeout
    if (tempStateTimer.isRunning() && tempStateTimer.hasElapsed(TEMP_STATE_TIME_SECONDS)) {
      tempStateTimer.stop();
    }

    if (!tempStateTimer.isRunning()) {
      // Determine the appropriate non-temp state
      if (DriverStation.isDisabled()) {
        state = LEDState.DISABLED;
      } else if (shooting) {
        state = LEDState.SHOOTING;
      } else if (intaking) {
        state = LEDState.INTAKING;
      } else {
        state = DriverStation.isAutonomous() ? LEDState.AUTONOMOUS : LEDState.TELEOP;
      }
    }

    // Blink timing
    if (state != lastState) {
      if (state.blink) { blinkOff = false; blinkTimer = 0; } else { blinkOff = false; }
    }
    if (state.blink) {
      double rate = (state == LEDState.ERROR || state == LEDState.WARNING || state == LEDState.INFO)
          ? ERROR_BLINK_RATE_SECONDS : BLINK_RATE_SECONDS;
      blinkTimer += loopDt;
      if (blinkTimer >= rate) { blinkTimer = 0; blinkOff = !blinkOff; }
    }

    renderState(state);
    if (state == LEDState.AUTONOMOUS || state == LEDState.TELEOP) {
      renderConfidenceOverlay();
    }
    lastState = state;
  }

  private void renderState(LEDState s) {
    if (blinkOff) { allOff(); return; }

    switch (s) {
      case STARTUP:
      case DISABLED:
        fireActive = true; // fall through to fire on next loop
        break;

      case AUTONOMOUS:
        fireOnSides(Palette.RED, 212, 42, 0);
        break;

      case TELEOP:
        fireOnSides(Palette.GREEN_PURPLE, 0, 200, 0);
        break;

      case SHOOTING: {
        double target = Math.max(targetFlywheelVelocity, 1.0e-9);
        double progress = Math.min(1.0, currentFlywheelVelocity / target);
        if (progress >= 0.95) {
          allSolid(0, 255, 0);
        } else {
          progressBar(progress, 0, 255, 0);
        }
        break;
      }

      case INTAKING:
        candleGroup.setLEDs(0, 100, 150, 0, 40);
        candleGroup.setLEDs(0, 100, 150, 48, 40);
        break;

      case ERROR:
        allSolid(255, 0, 0);
        break;
      case WARNING:
        allSolid(255, 200, 0);
        break;
      case INFO:
        allSolid(200, 200, 255);
        break;
    }
  }

  private void renderConfidenceOverlay() {
    // Not rendered over CANdle in the initial pass — reserved for future use.
  }
}