package frc.robot.Subsystems;

import Glitch.Lib.LEDs.AbstractLEDS;
import Glitch.Lib.LEDs.GlitchLEDPatterns;

public class LEDSubsystem extends AbstractLEDS {

    Section firstHalf;
    Section secondHalf;

    public LEDSubsystem() {
        super(100, 50, -50);

        firstHalf = getSections().get(0);
        firstHalf.setBase(GlitchLEDPatterns.fire(GlitchLEDPatterns.theCoolerGreen));

        secondHalf = getSections().get(1);
        secondHalf.setBase(GlitchLEDPatterns.ripple(GlitchLEDPatterns.funGradient, 2, 1));
    }
    
    public void start() {
        firstHalf.setPattern(GlitchLEDPatterns.purple, 2);
        secondHalf.setPattern(GlitchLEDPatterns.purple, 2);
    }
}
