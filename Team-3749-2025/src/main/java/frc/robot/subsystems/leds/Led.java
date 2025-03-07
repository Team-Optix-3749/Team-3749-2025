package frc.robot.subsystems.leds;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Percent;

import java.util.Optional;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.leds.LEDConstants.LEDColor;
import edu.wpi.first.wpilibj.LEDPattern;

import frc.robot.subsystems.leds.real.LedReal;
import frc.robot.subsystems.leds.sim.LedSim;

public class Led extends SubsystemBase {
    private AddressableLEDBuffer ledBuffer;

    private LEDColor desiredPattern = getTeamColorLED();
    private LEDColor currentPattern = null;

    // private StatusIndicator statusIndicator = StatusIndicator.TEAM;

    private double brightness = 1;

    private final LEDPattern rainbowPattern = LEDPattern.rainbow(255, 128);
    private final Distance ledSpacing = Meters.of(Units.inchesToMeters(11.5) / 18);
    private final LEDPattern scrollingRainbow = rainbowPattern.scrollAtAbsoluteSpeed(MetersPerSecond.of(0.5),
            ledSpacing);
    private LEDIO ledBase;

    public Led() {
        ledBuffer = new AddressableLEDBuffer(LEDConstants.length);

        if (Robot.isReal()) {
            ledBase = new LedReal(LEDConstants.ledPort, ledBuffer);
        } else {
            ledBase = new LedSim(LEDConstants.ledPort, ledBuffer);
        }
        // ledBase.setData(LEDPattern.kOff);
        LedTriggers.createLEDTriggers();
    }

    /**
     * Takes the parameter of brightness to set the brightness of the LEDs
     * 
     * @param brightness
     */
    public Led(double brightness) {
        this();
        this.brightness = brightness;
    }

    /**
     * Returns a LED pattern that matches the current alliance of the robot
     * 
     * @return
     */
    public LEDColor getTeamColorLED() {
        Optional<Alliance> team = DriverStation.getAlliance(); // i hate doing it this way but it throws an error
                                                               // without it
        if (!team.isPresent()) {
            return LEDColor.NO_TEAM;
        }

        return team.get() == Alliance.Blue ? LEDColor.BLUE_ALLIANCE : LEDColor.RED_ALLIANCE;
    }

    private void setStripColor() {
        if (this.desiredPattern == LEDColor.RAINBOW) {
            ledBase.setData(scrollingRainbow);
            currentPattern = desiredPattern;
            return;
        }
        if (desiredPattern == currentPattern) {
            return;
        }

        LEDPattern setPattern = LEDPattern.solid(desiredPattern.color).atBrightness(Percent.of(brightness * 100));
        ledBase.setData(setPattern);

        currentPattern = desiredPattern;
    }

    public void setLEDColor(LEDColor color) {
        this.desiredPattern = color;
    }

    // public void setLEDStatusIndicator(StatusIndicator indicator) {
    //     statusIndicator = indicator;
    // }

    /**
     * Returns the current LED pattern
     * 
     * @return
     */
    public LEDColor getCurrentPattern() {
        return currentPattern;
    }

    /***
     * 
     * @param setBrightness value from 1 - 0 for brightness
     */
    public void setBrightness(double setBrightness) {
        brightness = setBrightness;
    }

    @Override
    public void periodic() {

    }

}