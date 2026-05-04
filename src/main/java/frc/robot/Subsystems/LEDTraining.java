package frc.robot.Subsystems;

import java.util.ArrayList;

import Glitch.Lib.LEDs.AbstractLEDS;
import Glitch.Lib.LEDs.GlitchLEDInterfaces.LEDArrayPattern;
import Glitch.Lib.LEDs.GlitchLEDInterfaces.TwoDArrayPattern;
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

    public class MyTestPattern implements LEDArrayPattern {
        ArrayList<Color> colors = new ArrayList<Color>();

        public ArrayList<Color> colorList(ArrayList<Color> colorList) {
            return colors;
        }
    }

    public class MyTwoDPattern extends TwoDArrayPattern {
        public MyTwoDPattern(int width, int height, StartPosition physicalStart, StartPosition intendedOrigin, Alignment alignment, boolean serpentine) {
            super(width, height, physicalStart, intendedOrigin, alignment, serpentine);
        }

        public ArrayList<Color> colorList(ArrayList<Color> colorList) {
            return twoDToOneDConverter(this.arrayList);
        }
    }

    public MyTwoDPattern twoDPattern = new MyTwoDPattern(
        10, 10,
        TwoDArrayPattern.StartPosition.TOP_LEFT,
        TwoDArrayPattern.StartPosition.TOP_LEFT,
        TwoDArrayPattern.Alignment.ROW_MAJOR,
        true
    );
}