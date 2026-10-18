package frc.lib.util;

import java.util.Random;

import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.signals.AnimationDirectionValue;
import com.ctre.phoenix6.signals.RGBWColor;

import edu.wpi.first.wpilibj.Timer;

/**
 * Software-driven fire animation supporting selectable color palettes.
 *
 * <p>Port of Mark Kriegsman's "Fire2012" algorithm tuned to visually match CTRE Phoenix 6's
 * built-in FireAnimation, with configurable color palettes. Added WHITE palette for team colors.
 *
 * <p>Ported and adapted from Lynk Robotics (Team 9496) and original Glitch 2.0 implementation.
 */
public class CustomFireAnimation {
    public enum Palette {
        /** Black -> Red -> Yellow -> White */
        RED,
        /** Black -> Blue -> Cyan -> White */
        BLUE,
        /** Black -> Green -> Purple -> White */
        GREEN_PURPLE,
        /** Black -> White -> Bright White (for Glitch white theme) */
        WHITE,
        /** Black -> Green -> Bright White (white-green theme) */
        WHITE_GREEN,
        /** Black -> Green -> Cyan -> White (teal-green theme) */
        TEAL_GREEN
    }

    private static final int QUANT_LEVELS = 8;

    private final int startIdx;
    private final int numLEDs;

    private final int[] heat;
    private final int[] lastLevel;

    private final RGBWColor[] palettesByLevel;

    private final Timer frameTimer = new Timer();
    private final Random random = new Random();

    private double frameRate = 60.0;
    private double brightness = 1.0;
    private double sparking = 0.5;
    private double cooling = 0.5;
    private AnimationDirectionValue direction = AnimationDirectionValue.Forward;
    private Palette palette = Palette.WHITE_GREEN;

    private boolean palettesDirty = true;
    private boolean forceFullSend = true;

    public CustomFireAnimation(int ledStartIndex, int ledEndIndex) {
        this.startIdx = ledStartIndex;
        this.numLEDs = ledEndIndex - ledStartIndex + 1;
        this.heat = new int[numLEDs];
        this.lastLevel = new int[numLEDs];
        this.palettesByLevel = new RGBWColor[QUANT_LEVELS];
        for (int i = 0; i < numLEDs; i++) {
            lastLevel[i] = -1;
        }
        frameTimer.start();
    }

    public CustomFireAnimation withFrameRate(double frameRate) {
        this.frameRate = frameRate;
        return this;
    }

    public CustomFireAnimation withBrightness(double brightness) {
        if (this.brightness != brightness) {
            this.brightness = brightness;
            this.palettesDirty = true;
            this.forceFullSend = true;
        }
        return this;
    }

    public CustomFireAnimation withSparking(double sparking) {
        this.sparking = sparking;
        return this;
    }

    public CustomFireAnimation withCooling(double cooling) {
        this.cooling = cooling;
        return this;
    }

    public CustomFireAnimation withDirection(AnimationDirectionValue direction) {
        if (this.direction != direction) {
            this.direction = direction;
            this.forceFullSend = true;
        }
        return this;
    }

    public CustomFireAnimation withPalette(Palette palette) {
        if (this.palette != palette) {
            this.palette = palette;
            this.palettesDirty = true;
            this.forceFullSend = true;
        }
        return this;
    }

    public Palette getPalette() {
        return palette;
    }

    public void reset() {
        for (int i = 0; i < numLEDs; i++) {
            heat[i] = 0;
            lastLevel[i] = -1;
        }
        forceFullSend = true;
        frameTimer.restart();
    }

    public boolean update(CANdleGroup leds) {
        if (!frameTimer.hasElapsed(1.0 / frameRate)) return false;
        frameTimer.restart();

        if (palettesDirty) rebuildPalette();

        simulateStep();
        renderRunLengthEncoded(leds);
        return true;
    }

    private void simulateStep() {
        int coolMax = Math.max(2, (int) Math.round(cooling * 100.0 * 10.0 / numLEDs) + 2);
        for (int i = 0; i < numLEDs; i++) {
            heat[i] = Math.max(0, heat[i] - random.nextInt(coolMax));
        }

        for (int k = numLEDs - 1; k >= 2; k--) {
            heat[k] = (heat[k - 1] + heat[k - 2] + heat[k - 2]) / 3;
        }

        if (random.nextDouble() < sparking) {
            int y = random.nextInt(Math.min(7, numLEDs));
            heat[y] = Math.min(255, heat[y] + 160 + random.nextInt(96));
        }
    }

    private void renderRunLengthEncoded(CANdleGroup leds) {
        boolean reversed = direction == AnimationDirectionValue.Backward;

        int runStart = 0;
        int runLevel = quantize(heat[reversed ? numLEDs - 1 : 0]);

        for (int i = 1; i <= numLEDs; i++) {
            int level = (i == numLEDs) ? -1 : quantize(heat[reversed ? numLEDs - 1 - i : i]);
            if (level != runLevel) {
                flushRun(leds, runStart, i - 1, runLevel);
                runStart = i;
                runLevel = level;
            }
        }
        forceFullSend = false;
    }

    private void flushRun(CANdleGroup leds, int physStart, int physEnd, int level) {
        boolean changed = forceFullSend;
        if (!changed) {
            for (int j = physStart; j <= physEnd; j++) {
                if (lastLevel[j] != level) { changed = true; break; }
            }
        }
        if (!changed) return;

        SolidColor req = new SolidColor(startIdx + physStart, startIdx + physEnd)
            .withColor(palettesByLevel[level])
            .withUpdateFreqHz(0);
        leds.setControl(req);

        for (int j = physStart; j <= physEnd; j++) {
            lastLevel[j] = level;
        }
    }

    private int quantize(int heatVal) {
        if (heatVal <= 0) return 0;
        if (heatVal >= 255) return QUANT_LEVELS - 1;
        return (heatVal * QUANT_LEVELS) >> 8;
    }

    private void rebuildPalette() {
        for (int i = 0; i < QUANT_LEVELS; i++) {
            int heatVal = (i * 255) / (QUANT_LEVELS - 1);
            palettesByLevel[i] = heatToColor(heatVal, palette, brightness);
        }
        palettesDirty = false;
    }

    private static RGBWColor heatToColor(int heat, Palette palette, double brightness) {
        int t192 = (heat * 191) >> 8;
        int heatramp = (t192 & 0x3F) << 2;

        int r, g, b;

        switch (palette) {
            case WHITE_GREEN:
                // Black -> Green -> White -> Bright White
                if ((t192 & 0x80) != 0) {
                    // 128-191: white -> bright white
                    r = 255;
                    g = 255;
                    b = 255 - heatramp;
                } else if ((t192 & 0x40) != 0) {
                    // 64-127: green -> white
                    r = heatramp;
                    g = 255;
                    b = heatramp;
                } else {
                    // 0-63: black -> green
                    r = 0;
                    g = heatramp;
                    b = 0;
                }
                break;

            case WHITE:
                // Pure white fire: Black -> Dim White -> White -> Bright White
                if ((t192 & 0x80) != 0) {
                    r = 255;
                    g = 255;
                    b = 255;
                } else if ((t192 & 0x40) != 0) {
                    int v = 128 + heatramp;
                    r = v; g = v; b = v;
                } else {
                    int v = Math.min(heatramp * 2, 128);
                    r = v; g = v; b = v;
                }
                break;

            case TEAL_GREEN:
                // Black -> Green -> Teal/Cyan -> White
                if ((t192 & 0x80) != 0) {
                    r = heatramp;
                    g = 255;
                    b = 255;
                } else if ((t192 & 0x40) != 0) {
                    r = 0;
                    g = 255;
                    b = heatramp;
                } else {
                    r = 0;
                    g = heatramp;
                    b = 0;
                }
                break;

            case GREEN_PURPLE:
                if ((t192 & 0x80) != 0) {
                    r = 255;
                    g = 255 - heatramp;
                    b = 255;
                } else if ((t192 & 0x40) != 0) {
                    r = heatramp;
                    g = 255;
                    b = heatramp;
                } else {
                    r = 0;
                    g = heatramp;
                    b = 0;
                }
                break;

            case BLUE:
                if ((t192 & 0x80) != 0) {
                    r = 255; g = 255; b = heatramp;
                } else if ((t192 & 0x40) != 0) {
                    r = 0; g = heatramp; b = 255;
                } else {
                    r = 0; g = 0; b = heatramp;
                }
                break;

            case RED:
            default:
                if ((t192 & 0x80) != 0) {
                    r = 255; g = 255; b = heatramp;
                } else if ((t192 & 0x40) != 0) {
                    r = 255; g = heatramp; b = 0;
                } else {
                    r = heatramp; g = 0; b = 0;
                }
                break;
        }

        r = clamp((int) Math.round(r * brightness));
        g = clamp((int) Math.round(g * brightness));
        b = clamp((int) Math.round(b * brightness));

        return new RGBWColor(r, g, b, 0);
    }

    private static int clamp(int v) {
        if (v < 0) return 0;
        if (v > 255) return 255;
        return v;
    }
}