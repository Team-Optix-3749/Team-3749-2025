package frc.robot.subsystems.led;

import java.util.Optional;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.led.LEDConstants.LEDPattern;

/**
 * 
 * LED subsystem, can be different colors (RED, BLUE, GREEN, YELLOW, RAINBOW,
 * WHITE, and NOTHING)
 * The LEDs are used to indicate when the robot does different actions
 */
public class LED extends SubsystemBase {

    private AddressableLED LEDs = new AddressableLED(9); // port
    private AddressableLEDBuffer LEDBuffer = new AddressableLEDBuffer(frc.robot.subsystems.led.LEDConstants.length);
    private LEDPattern currentPattern = LEDPattern.WHITE;
    private int hue = 0;
    private double brightness = 1;

    public LED() {
        LEDs.setLength(LEDBuffer.getLength());
        LEDs.setData(LEDBuffer);
        LEDs.start();
        setBrightness(brightness);
        setLEDPattern(LEDPattern.WHITE);
    }

    /**
     * Takes the parameter of brightness to set the brightness of the LEDs
     * 
     * @param brightness
     */
    public LED(double brightness) {
        this.brightness = brightness;
        LEDs.setLength(LEDBuffer.getLength());
        LEDs.setData(LEDBuffer);
        LEDs.start();
        setLEDPattern(LEDPattern.WHITE);
        setBrightness(brightness);
    }

    /**
     * Returns a LED pattern that matches the current alliance of the robot
     * 
     * @return
     */
    private LEDPattern teamColorLED() {
        Optional<Alliance> team = DriverStation.getAlliance(); // i hate doing it this way but it throws an error
                                                               // without it
        return team.get() == Alliance.Blue ? LEDPattern.BLUE : LEDPattern.RED;
    }

    /**
     * Takes in the parameters R, G, and B to set the LEDs to one color using an RGB
     * color code
     * 
     * @param R
     * @param G
     * @param B
     */
    private void setLEDOneColorRGB(int R, int G, int B) {
        double curBrightness = brightness;
        if (DriverStation.isEnabled()) {
            curBrightness = 1;
        }
        int setR = (int) (R * curBrightness);
        int setG = (int) (G * curBrightness);
        int setB = (int) (B * curBrightness);

        for (int i = 0; i < LEDBuffer.getLength(); i++) {
            LEDBuffer.setRGB(i, setR, setG, setB);
        }
    }

    /**
     * Takes in the parameters H, S, and V to set the LEDs to one color using an HSV
     * color code
     * 
     * @param H
     * @param S
     * @param V
     */
    private void setLEDOneColorHSV(int H, int S, int V) {
        for (int i = 0; i < LEDBuffer.getLength(); i++) {
            LEDBuffer.setRGB(i, H, S, V);
        }
    }

    /**
     * Sets the LED color to Rainbow.
     * I think it might be easier to use LEDPattern.rainbow, but this works, too
     */
    private void setLEDRainbow() // requires a loop
    {
        hue++;
        setLEDOneColorHSV(hue, 255, 255);
        if (hue >= 350) {
            hue = 0;
        }
    }

    /**
     * Takes in the parameter pattern to set the pattern of the LEDs
     * If the pattern is WHITE and the battery voltage is less than 8, the
     * LEDpattern is set to RED.
     * 
     * @param pattern
     */
    public void setLEDPattern(LEDPattern pattern) {
        if (pattern == LEDPattern.WHITE && RobotController.getBatteryVoltage() < 8) {
            pattern = LEDPattern.RED;
        }
        this.currentPattern = pattern;
    }

    /**
     * Returns the current LED pattern
     * 
     * @return
     */
    public LEDPattern getCurrentPattern() {
        return currentPattern;
    }

    // runs every 0.02 sec
    @Override
    public void periodic() {
        switch (this.currentPattern) {
            case RED:
                setLEDOneColorRGB(255, 0, 0);
                break;

            case BLUE:
                setLEDOneColorRGB(0, 0, 255);
                break;

            case GREEN:
                setLEDOneColorRGB(0, 255, 0);
                break;

            case YELLOW:
                setLEDOneColorRGB(255, 255, 0);
                break;

            case RAINBOW:
                setLEDRainbow();
                break;

            case WHITE:
                setLEDOneColorRGB(255, 255, 255);
                break;

            case NOTHING:
                setLEDOneColorRGB(0, 0, 0);
                break;

            default:
                setLEDOneColorRGB(210, 105, 30);
                System.out.println("LEDpattern missing case");
                break;
        }
        LEDs.setData(LEDBuffer);

    }

    /***
     * 
     * @param setBrightness value from 1 - 0 for brightness
     */
    public void setBrightness(double setBrightness) {
        brightness = setBrightness;
    }
}