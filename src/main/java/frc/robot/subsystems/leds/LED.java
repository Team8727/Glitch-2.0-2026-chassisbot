package frc.robot.subsystems.leds;

import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.AnimationDirectionValue;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.CANdleGroup;
import frc.lib.util.CustomFireAnimation;
import frc.lib.util.CustomFireAnimation.Palette;
import frc.robot.Constants;

public class LED extends SubsystemBase {
    public static final LED instance = new LED();
    
    private static CANdleGroup candleGroup;
    private static final Timer tempStateTimer = new Timer();
    private static final Timer blinkTimer = new Timer();
    
    private static LEDState state = LEDState.STARTUP;
    private static LEDState lastState = null;
    private static boolean blinkOff = false;
    
    private static boolean shooting = false;
    private static boolean intaking = false;
    private static double currentFlywheelRPM = 0.0;
    private static double targetFlywheelRPM = 0.0;
    private static int visionMeasurementCount = 0;
    
    // Pre-built fire animations
    private static CustomFireAnimation disabledFire;
    private static CustomFireAnimation autoFire;
    private static CustomFireAnimation teleopFire;
    
    public enum LEDState {
        STARTUP, DISABLED, AUTONOMOUS, TELEOP, SHOOTING, INTAKING,
        ERROR, WARNING, INFO, VISION_LOST;
        
        public final boolean blink;
        public final double blinkRate;
        
        LEDState() { this(false, LEDConstants.kBlinkRate); }
        LEDState(boolean blink, double rate) { this.blink = blink; this.blinkRate = rate; }
    }
    
    private LED() {
        CANdle candle = new CANdle(Constants.kCANdleId);
        candleGroup = new CANdleGroup(candle);

        // Send an initial full-strip command so the CANdle learns we have 88 LEDs
        candleGroup.setLEDs(0, 0, 0, 0, Constants.kLEDTotal);
        
        disabledFire = new CustomFireAnimation(0, Constants.kLEDTotal - 1)
            .withFrameRate(LEDConstants.kFireFrameRate)
            .withSparking(LEDConstants.kFireSparking)
            .withCooling(LEDConstants.kFireCooling)
            .withBrightness(1.0)
            .withPalette(LEDConstants.kDisabledPalette)
            .withDirection(AnimationDirectionValue.Forward);
        
        autoFire = new CustomFireAnimation(0, Constants.kLEDTotal - 1)
            .withFrameRate(LEDConstants.kFireFrameRate)
            .withSparking(0.7).withCooling(0.5)
            .withBrightness(1.0)
            .withPalette(LEDConstants.kAutoPalette)
            .withDirection(AnimationDirectionValue.Forward);
        
        teleopFire = new CustomFireAnimation(0, Constants.kLEDTotal - 1)
            .withFrameRate(LEDConstants.kFireFrameRate)
            .withSparking(0.7).withCooling(0.5)
            .withBrightness(1.0)
            .withPalette(LEDConstants.kTeleopPalette)
            .withDirection(AnimationDirectionValue.Forward);
    }
    
    // ========== Public API ==========
    
    public void start() { state = LEDState.DISABLED; }
    public void autoInit() { shooting = false; intaking = false; state = LEDState.AUTONOMOUS; }
    public void teleopInit() { shooting = false; intaking = false; state = LEDState.TELEOP; }
    
    public void updateVisionConfidence(int count) { visionMeasurementCount = count; }
    
    public void shootPatterns(double currentRPM, double targetRPM) {
        shooting = true;
        currentFlywheelRPM = currentRPM;
        targetFlywheelRPM = targetRPM;
    }
    
    public void endCommand() { shooting = false; }
    public void intakePatterns() { intaking = !intaking; }
    
    public void triggerError() { tempStateTimer.restart(); state = LEDState.ERROR; }
    public void triggerWarning() { tempStateTimer.restart(); state = LEDState.WARNING; }
    public void triggerInfo() { tempStateTimer.restart(); state = LEDState.INFO; }
    
    // ========== Periodic ==========
    
    @Override
    public void periodic() {
        if (candleGroup == null) return;
        
        // Determine base state
        if (tempStateTimer.isRunning() && tempStateTimer.hasElapsed(LEDConstants.kTempStateTime)) {
            tempStateTimer.stop();
        }
        
        if (tempStateTimer.isRunning()) {
            // Keep the temp state
        } else if (DriverStation.isDisabled()) {
            state = LEDState.DISABLED;
        } else if (shooting) {
            state = LEDState.SHOOTING;
        } else if (intaking) {
            state = LEDState.INTAKING;
        } else if (visionMeasurementCount == 0 && state == LEDState.TELEOP) {
            state = LEDState.VISION_LOST;
        } else {
            state = DriverStation.isAutonomous() ? LEDState.AUTONOMOUS : LEDState.TELEOP;
        }
        
        // Blink timing
        boolean blinkToggle = false;
        if (state != lastState) {
            if (state.blink) { blinkOff = false; blinkTimer.restart(); }
            else { blinkTimer.stop(); }
        } else {
            double rate = (state == LEDState.ERROR || state == LEDState.WARNING) 
                ? LEDConstants.kErrorBlinkRate : LEDConstants.kBlinkRate;
            if (blinkTimer.isRunning() && blinkTimer.advanceIfElapsed(rate)) {
                blinkOff = !blinkOff;
                blinkToggle = true;
            }
        }
        
        renderState(state, blinkOff, blinkToggle);
        lastState = state;
    }
    
    private void renderState(LEDState s, boolean blinkOff, boolean blinkToggle) {
        if (s.blink && blinkOff) { allOff(); return; }
        
        switch (s) {
            case STARTUP:
            case DISABLED:
                disabledFire.update(candleGroup);
                break;
                
            case AUTONOMOUS:
                autoFire.update(candleGroup);
                break;
                
            case TELEOP:
                teleopFire.update(candleGroup);
                break;
                
            case SHOOTING: {
                double target = Math.max(targetFlywheelRPM, 1.0);
                double progress = Math.min(1.0, currentFlywheelRPM / target);
                if (progress >= 0.95) {
                    allSolid(0, 255, 0);
                } else {
                    progressBar(progress, 0, 255, 0);
                }
                break;
            }
            
            case INTAKING:
                candleGroup.setLEDs(255, 255, 255, 0, 40);
                candleGroup.setLEDs(255, 255, 255, 48, 40);
                break;
                
            case VISION_LOST:
                candleGroup.setLEDs(255, 255, 200, 0, 40);
                candleGroup.setLEDs(200, 200, 255, 48, 40);
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
    
    private void allOff() { candleGroup.setLEDs(0, 0, 0, 0, Constants.kLEDTotal); }
    private void allSolid(int r, int g, int b) { candleGroup.setLEDs(r, g, b, 0, Constants.kLEDTotal); }
    
    private void progressBar(double pct, int r, int g, int b) {
        int lit = (int) Math.round(Constants.kLEDTotal * pct);
        if (lit > 0) candleGroup.setLEDs(r, g, b, 0, lit);
        int unlit = Constants.kLEDTotal - lit;
        if (unlit > 0) candleGroup.setLEDs(0, 0, 0, lit, unlit);
    }
}