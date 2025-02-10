package frc.robot.subsystems.leds;

import java.util.Optional;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.leds.LEDConstants.LEDPattern;

/**
 * 
 * LED subsystem, can be different colors (RED, BLUE, GREEN, YELLOW,
 * WHITE, and NOTHING)
 * The LEDs are used to indicate when the robot does different actions
 */
public class LEDs extends SubsystemBase {

    private AddressableLED LED1 = new AddressableLED(0); // port
    // private AddressableLED LED2 = new AddressableLED(1); // port

    private AddressableLEDBuffer LEDBuffer = new AddressableLEDBuffer(LEDConstants.length);
    private LEDPattern currentPattern = LEDPattern.WHITE;
    private int hue = 0;
    private double brightness = 1;

    public LEDs() {
        LED1.setLength(LEDBuffer.getLength());
        LED1.setData(LEDBuffer);
        LED1.start();
        // LED2.setLength(LEDBuffer.getLength());
        // LED2.setData(LEDBuffer);
        // LED2.start();
        setBrightness(brightness);
        setLEDPattern(LEDPattern.WHITE);
    }

    /**
     * Takes the parameter of brightness to set the brightness of the LEDs
     * 
     * @param brightness
     */
    public LEDs(double brightness) {
        this.brightness = brightness;
        LED1.setLength(LEDBuffer.getLength());
        LED1.setData(LEDBuffer);
        LED1.start();
        // LED2.setLength(LEDBuffer.getLength());
        // LED2.setData(LEDBuffer);
        // LED2.start();
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

    /***
     * 
     * @param setBrightness value from 1 - 0 for brightness
     */
    public void setBrightness(double setBrightness) {
        brightness = setBrightness;
    }

    // runs every 0.02 sec
    @Override
    public void periodic() {

        setLEDOneColorRGB(this.currentPattern.R, this.currentPattern.G, this.currentPattern.B);

        // if (!Robot.elevator.getIsStableState() || !Robot.coralArm.getIsStableState()) {
        //     setLEDPattern(LEDPattern.YELLOW);
        // } else if (Robot.coralRoller.hasPiece()) {
        //     setLEDPattern(LEDPattern.BLUE);
        // } else if (Robot.scoringRoller.hasPiece()) {
        //     setLEDPattern(LEDPattern.GREEN);
        // } else {
        //     led.setLEDPattern(LEDPattern.WHITE);
        // }

        LED1.setData(LEDBuffer);
        // LED2.setData(LEDBuffer);

    }

}