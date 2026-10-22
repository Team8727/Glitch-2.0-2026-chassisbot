package frc.lib.util;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.EmptyAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.Enable5VRailValue;
import com.ctre.phoenix6.signals.LossOfSignalBehaviorValue;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StripTypeValue;

/**
 * Wraps one or more CTRE CANdles controlling a contiguous LED strip.
 *
 * <p>Provides helpers to set colors and clear animations. Colors are sent over CAN as
 * coalesced SolidColor frames. Designed to keep CAN bus usage low.
 *
 * <p>Ported and adapted from Lynk Robotics (Team 9496) — thanks Lynk!
 */
public class CANdleGroup {
    private final CANdle[] candles;
    private final ControlRequest clearAnimation = new EmptyAnimation(0);

    public CANdleGroup(CANdle... candles) {
        this.candles = candles;
        applyConfigs();
    }

    private void applyConfigs() {
        CANdleConfiguration config = new CANdleConfiguration();
        config.LED.BrightnessScalar = 1.0;
        config.LED.StripType = StripTypeValue.RGB; // Strip expects RGB byte order (matches old PWM ColorOrder.kRGB).
        config.LED.LossOfSignalBehavior = LossOfSignalBehaviorValue.KeepRunning;
        config.CANdleFeatures.Enable5VRail = Enable5VRailValue.Enabled;

        for (CANdle candle : candles) {
            candle.getConfigurator().apply(config);
        }
    }

    public void animate(ControlRequest animation) {
        for (CANdle candle : candles) {
            candle.setControl(animation);
        }
    }

    /** Pass a ControlRequest to every CANdle without logging. For high-frequency callers. */
    public void setControl(ControlRequest request) {
        for (CANdle candle : candles) {
            candle.setControl(request);
        }
    }

    public void clearAnimation() {
        for (CANdle candle : candles) {
            candle.setControl(clearAnimation);
        }
    }

    /**
     * Sets a contiguous range of LEDs to a single color.
     *
     * @param color    RGBW color
     * @param startIdx first LED index (inclusive, 0-based)
     * @param count    number of LEDs to set
     */
    public void setLEDs(RGBWColor color, int startIdx, int count) {
        if (count <= 0) return;
        for (CANdle candle : candles) {
            candle.setControl(
                new SolidColor(startIdx, startIdx + count - 1).withColor(color));
        }
    }

    /** Convenience overload without white channel. */
    public void setLEDs(int r, int g, int b, int startIdx, int count) {
        setLEDs(new RGBWColor(r, g, b, 0), startIdx, count);
    }
}