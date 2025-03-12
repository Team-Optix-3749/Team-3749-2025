package frc.robot.subsystems.leds;

import static edu.wpi.first.units.Units.Percent;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;

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

    private LEDColor desiredColor;
    private LEDColor currentColor = null;
    
    private double brightness = 1;

    private LEDIO ledBase;

    public Led() {
        ledBuffer = new AddressableLEDBuffer(LEDConstants.length);

        if (Robot.isReal()) {
            ledBase = new LedReal(LEDConstants.ledPort, ledBuffer);
        } else {
            ledBase = new LedSim(LEDConstants.ledPort, ledBuffer);
        }
        desiredColor = getTeamColorLED();
        ledBase.setData(LEDPattern.kOff);
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
     * Returns the current LED pattern
     * 
     * @return
     */
    public LEDColor getCurrentColor() {
        return currentColor;
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

        return team.get() == Alliance.Red ? LEDColor.RED_ALLIANCE : LEDColor.BLUE_ALLIANCE;
    }

    public void setColor(LEDColor color) {
        this.desiredColor = color;
    }

    /***
     * 
     * @param setBrightness value from 1 - 0 for brightness
     */
    public void setBrightness(double setBrightness) {
        brightness = setBrightness;
    }

    public void updateLEDs() {

        if (desiredColor == LEDColor.RAINBOW) {
            LEDPattern setPattern = LEDConstants.scrollingRainbow;
            ledBase.setData(setPattern);
            currentColor = desiredColor;
            return;
        }

        LEDPattern setPattern = LEDPattern.solid(desiredColor.color).atBrightness(Percent.of(brightness * 100));

      

        if (desiredColor == LEDColor.RED_ALLIANCE || desiredColor == LEDColor.BLUE_ALLIANCE) {
            setPattern = LEDPattern.solid(getTeamColorLED().color).atBrightness(Percent.of(brightness * 100));
        }

        ledBase.setData(setPattern);
        currentColor = desiredColor;
    }

    public void logData() {
        Logger.recordOutput("LED/currentPattern", currentColor.toString());
        Logger.recordOutput("LED/desiredPattern", desiredColor.toString());
        Logger.recordOutput("LED/brightness", brightness);
    }

    @Override
    public void periodic() {
        updateLEDs();
        logData();
    }

}