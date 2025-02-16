package frc.robot.utils;

import java.util.Map;
import java.util.function.Consumer;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.Seconds;

/**
 * SysId assisted control tuning
 * 
 * @author Dhyan Soni
 */
public class SysIdTuner {
    private SysIdRoutine sysIdRoutine;
    private Consumer<SysIdRoutineLog> setLog;
    private Consumer<Voltage> setVolts;

    private VoltageDrive io;
    private Map<String, MotorData> motorData;

    private Type type;

    private SysIdRoutine.Config config = new SysIdRoutine.Config(
            Volts.per(Seconds).of(1), // Voltage ramp rate
            Volts.of(7), // Max voltage
            Seconds.of(4) // Test duration
    );

    /**
     * Setup SysId
     * 
     * @param name      - name of the mechanism
     * @param subsystem - Subsystem you are characterizing
     * @param io        - VoltageDrive
     */
    public SysIdTuner(String name, Subsystem subsystem, VoltageDrive io,
            Map<String, MotorData> motorData, Type type) {

        this.io = io;
        this.motorData = motorData;

        setVolts = (Voltage volts) -> setVoltage(volts);
        setLog = (SysIdRoutineLog) -> setLog(SysIdRoutineLog);

        sysIdRoutine = new SysIdRoutine(
                config,
                new SysIdRoutine.Mechanism(setVolts, setLog, subsystem, name));

        this.type = type;
    }

    /**
     * Setup SysId
     * 
     * @param name      - name of the mechanism
     * @param config    - SysIdRoutine.Config object
     * @param subsystem - Subsystem you are characterizing
     * @param io        - VoltageDrive
     */
    public SysIdTuner(String name, SysIdRoutine.Config config, Subsystem subsystem, VoltageDrive io,
            Map<String, MotorData> motorData, Type type) {

        this.io = io;
        this.motorData = motorData;

        setVolts = (Voltage volts) -> setVoltage(volts);
        setLog = (SysIdRoutineLog) -> setLog(SysIdRoutineLog);

        sysIdRoutine = new SysIdRoutine(
                config,
                new SysIdRoutine.Mechanism(setVolts, setLog, subsystem, name));
        this.type = type;

    }

    /**
     * Update logs with voltage, linearPosition, linearVelocity, and
     * linearAcceleration
     * 
     * @param log - SysIdRoutineLog object
     */
    private void setLog(SysIdRoutineLog log) {
        if (type == Type.LINEAR) {
            motorData.forEach((motorName, data) -> {
                log.motor(motorName)
                        .voltage(Voltage.ofBaseUnits(data.appliedVolts, Volts))
                        .linearPosition(Meters.ofBaseUnits(data.position))
                        .linearVelocity(MetersPerSecond.ofBaseUnits(data.velocity))
                        .linearAcceleration(MetersPerSecondPerSecond.ofBaseUnits(data.acceleration));
            });
        } else {
            motorData.forEach((motorName, data) -> {
                log.motor(motorName)
                        .voltage(Voltage.ofBaseUnits(data.appliedVolts, Volts))
                        .angularPosition(Radians.ofBaseUnits(data.position))
                        .angularVelocity(RadiansPerSecond.ofBaseUnits(data.velocity))
                        .angularAcceleration(RadiansPerSecondPerSecond.ofBaseUnits(data.acceleration));
            });
        }

    }

    /**
     * Set voltage method in the Voltage unit
     * 
     * @param volts - voltage to be applied
     */
    public void setVoltage(Voltage volts) {
        try {
            io.setVoltage(volts.magnitude());
        } catch (Exception e) {
            System.err.println("Failed to set voltage: " + e.getMessage());
        }
    }

    /*
     * Used as the type for all subsystem IOs
     */
    public interface VoltageDrive {
        void setVoltage(double volts);
    }

    /**
     * Command for Quasistatic test
     * 
     * @return command and direction for Quasistatic test
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.quasistatic(direction);
    }

    /**
     * Command for Dynamic test
     * 
     * @return command and direction for Dynamic test
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.dynamic(direction);
    }

    /**
     * Command for running all tests
     * 
     * @return commands for SysId characterization
     */
    public Command runTests() {
        // need to set burnout timer to lower values
        return sysIdRoutine.quasistatic(Direction.kForward).andThen(sysIdRoutine.quasistatic(Direction.kReverse)
                .andThen(sysIdRoutine.dynamic(Direction.kForward).andThen(sysIdRoutine.dynamic(Direction.kReverse))));
    }

    public enum Type {
        ROTATIONAL,
        LINEAR;
    }
}
