package frc.robot.subsystems.leds;

public class LEDConstants {

    public static final int length = 18;

    public static enum LEDPattern {
        RED(255, 0, 0), 
        BLUE(0, 0, 255), 
        GREEN(0, 255, 0), 
        YELLOW(255, 255, 0), 
        WHITE(255, 255, 255), 
        NOTHING(0, 0, 0);

        final int R;
        final int G;
        final int B;

        /**
         * Takes in the parameters of R, G, and B to set the RGB color of a color in the enum
         * 
         * @param R
         * @param G
         * @param B
         */
        LEDPattern (int R, int G, int B){
            this.R = R;
            this.G = G;
            this.B = B;
        }
    }

}