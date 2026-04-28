package frc.robot.Subsystems;

import Glitch.Lib.LEDs.AbstractLEDS;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;

public class LEDTraining extends AbstractLEDS{

    public Section ledStrip;

    public LEDTraining() {
        super(100, 100);

        ledStrip = getSections().get(0);
        ledStrip.setBase(LEDPattern.kOff);
    }

    // this is probably not necessary for the purposes of the training but it is going to be used in actual code
    private static LEDTraining instance;
        public static LEDTraining getInstance() {
            if (instance == null) {
                instance = new LEDTraining();
            }
            return instance;
        }

    @Override
    public void periodic() {
        super.periodic();
    }













    
    // Here is where the student will edit!
    public Color stripColor(double time) {
        int r = 100;
        int g = 0;
        int b = 50;

        return new Color(r, g, b);
    }
}