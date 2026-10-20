package frc.robot.subsystems.leds;

import frc.lib.util.CustomFireAnimation;

public final class LEDConstants {
    private LEDConstants() {}
    
    public static final double kTempStateTime = 0.70;
    public static final double kBlinkRate = 0.2;
    public static final double kErrorBlinkRate = 0.1;
    
    // Fire animation defaults
    public static final double kFireFrameRate = 60.0;
    public static final double kFireSparking = 0.8;
    public static final double kFireCooling = 0.65;
    
    // Custom palettes for white-green theme
    public static final CustomFireAnimation.Palette kDisabledPalette = CustomFireAnimation.Palette.WHITE_GREEN;
    public static final CustomFireAnimation.Palette kAutoPalette = CustomFireAnimation.Palette.WHITE_GREEN;
    public static final CustomFireAnimation.Palette kTeleopPalette = CustomFireAnimation.Palette.TEAL_GREEN;
}