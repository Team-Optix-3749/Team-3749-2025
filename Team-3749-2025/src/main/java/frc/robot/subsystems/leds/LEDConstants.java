package frc.robot.subsystems.leds;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;

public class LEDConstants {

    public static final int length = 18;
    public static final int ledPort = 0;

    private static final Distance ledSpacing = Meters.of(Units.inchesToMeters(11.5) / 18);

    public static final LEDPattern rainbowPattern = LEDPattern.rainbow(255, 128);
    public static final LEDPattern scrollingRainbow = rainbowPattern.scrollAtAbsoluteSpeed(MetersPerSecond.of(.5),
            ledSpacing);

    public static enum LEDColor {
        RED_ALLIANCE(Color.kRed),
        BLUE_ALLIANCE(Color.kBlue),
        CLIMB(Color.kDarkGreen),
        // SCORING(Color.kDeepPink),
        NO_TEAM(Color.kWhite),
        // CHUTE_HAS_PIECE(Color.kGreen),
        CORAL_ARM_HAS_PIECE(Color.kBlue),
        OFF(Color.kBlack),
        BATTERY_LOW(Color.kOrange),
        BATTERY_GOOD(Color.kWhite),
        RAINBOW(Color.kTomato), // this is not the actual color i just need to satisfy reqs

        L1(Color.kPurple),
        L2(Color.kLightBlue),
        L3(Color.kGold),
        L4(Color.kMaroon),

        ALGAE(Color.kAquamarine);

        Color color;

        LEDColor(Color color) {
            this.color = color;
        }
    }
}