package frc.robot.subsystems.swerve;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.*;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.swerve.SwerveConstants.ControlConstants;
import frc.robot.subsystems.swerve.SwerveModuleIO.ModuleData;
import frc.robot.utils.LoggedTunableNumber;

/**
 * General class for swerve modules that interacts with the
 * interface. Handles all logic relating to individual modules
 * 
 * @author Noah Simon
 * @author Rohin Sood
 * @author Raadwan Masum
 */
public class SwerveModule {

    private String name;
    private SwerveModuleState desiredState = new SwerveModuleState();

    private final SimpleMotorFeedforward drivingFeedFordward;

    private ModuleData moduleData = new ModuleData();
    private SwerveModuleIO moduleIO;

    private double previousSetpointVelocity = 0;


    private LoggedTunableNumber driveSpeed;
    private LoggedTunableNumber drivePosition;
    private LoggedTunableNumber driveAcceleration;
    private LoggedTunableNumber driveTemp;
    private LoggedTunableNumber driveVolts;
    private LoggedTunableNumber driveCurrent;

    private LoggedTunableNumber turningSpeed;
    private LoggedTunableNumber turningPosition;
    private LoggedTunableNumber turningTemp;
    private LoggedTunableNumber turningVolts;
    private LoggedTunableNumber turningCurrent;

    public SwerveModule(int index, SwerveModuleIO SwerveModule) {

        moduleIO = SwerveModule;

        if (index == 0) {
            name = "FL module";
        } else if (index == 1) {
            name = "FR module";
        } else if (index == 2) {
            name = "BL module";
        } else if (index == 3) {
            name = "BR module";
        }
        // Tab, name, data
        driveSpeed = new LoggedTunableNumber("Swerve/" + name + "/drive speed",
                moduleData.driveVelocityMPerSec);
        drivePosition = new LoggedTunableNumber("Swerve/" + name + "/drive position",
                moduleData.driveVelocityMPerSec);
        driveAcceleration = new LoggedTunableNumber("Swerve/" + name + "/drive position",
                moduleData.driveVelocityMPerSec);
        driveTemp = new LoggedTunableNumber("Swerve/" + name + "/drive temp",
                moduleData.driveVelocityMPerSec);
        driveVolts = new LoggedTunableNumber("Swerve/" + name + "/drive volts",
                moduleData.driveVelocityMPerSec);
        driveCurrent = new LoggedTunableNumber("Swerve/" + name + "/drive current",
                moduleData.driveVelocityMPerSec);

        turningSpeed = new LoggedTunableNumber("Swerve/" + name + "/turning speed",
                moduleData.driveVelocityMPerSec);
        turningPosition = new LoggedTunableNumber("Swerve/" + name + "/turning position",
                moduleData.driveVelocityMPerSec);
        turningTemp = new LoggedTunableNumber("Swerve/" + name + "/turning temp",
                moduleData.driveVelocityMPerSec);
        turningVolts = new LoggedTunableNumber("Swerve/" + name + "/turning volts",
                moduleData.driveVelocityMPerSec);
        turningCurrent = new LoggedTunableNumber("Swerve/" + name + "/turning current",
                moduleData.turnCurrentAmps);

        drivingFeedFordward = new SimpleMotorFeedforward(ControlConstants.kSDriving,
                ControlConstants.kVDriving, ControlConstants.kADriving);
    }

    public String getName() {
        return name;
    }

    /**
     * State has a velocity (m/s) and an angle(0-2pi) component
     * 
     * @return The state of the module as a SwerveModuleState object
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(
                moduleData.driveVelocityMPerSec,
                new Rotation2d(moduleData.turnPositionRad));
    }

    /**
     * State has a distance (m) and an angle (0-2pi) component
     * 
     * @return The position of the module as a SwerveModuleState object
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(moduleData.drivePositionM, new Rotation2d(moduleData.turnPositionRad));
    }

    /**
     * 
     * @return SwerveModuleState - Angle and velocity point
     */
    public SwerveModuleState getDesiredState() {
        return desiredState;
    }

    /**
     * sets the module to a given setpoint state
     * 
     * @param state the SwerveModule state, velocity (m/s) and angle(0-2pi) that
     *              will become the module's setpoint
     */
    public void setDesiredState(SwerveModuleState state) {

        state.optimize(getState().angle);

        // prevent micromovements on the motor
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            state.speedMetersPerSecond = 0;
        }

        this.desiredState = state;

        setDriveSpeed(state.speedMetersPerSecond);
        setTurnPosition(state.angle.getRadians());

    }

    /**
     * 
     * @param speedMetersPerSecond - the drive speed setpoint for the module
     */
    public void setDriveSpeed(double speedMetersPerSecond) {

        double setpointAcceleration = (speedMetersPerSecond - previousSetpointVelocity) / 0.02;
        previousSetpointVelocity = speedMetersPerSecond;

        double feedforward = drivingFeedFordward.calculate(speedMetersPerSecond, setpointAcceleration);

        moduleIO.setDriveVelocity(speedMetersPerSecond, feedforward);

    }

    /**
     * 
     * @param positionRad - the angle setpoint (0-2pi) for the module
     */
    public void setTurnPosition(double positionRad) {
        moduleIO.setTurnPosition(positionRad, 0);
    }

    public void setDriveVoltage(double volts) {
        moduleIO.setDriveVoltage(volts);

    }

    public void setTurnVoltage(double volts) {
        moduleIO.setTurnVoltage(volts);
    }

    public void setBreakMode(boolean enabled) {
        moduleIO.setDriveBrakeMode(enabled);
        moduleIO.setTurningBrakeMode(enabled);

    }

    public void stop() {
        // setDriveVoltage(0);
        // setTurnVoltage(0);
    }

    public ModuleData getModuleData() {
        return moduleData;
    }

    // called within the swerve subsystem's periodic
    public void periodic() {
        moduleIO.updateData(moduleData);
        // // Logging
        driveSpeed.set(moduleData.driveVelocityMPerSec);
        drivePosition.set(moduleData.drivePositionM);
        driveAcceleration.set(moduleData.driveAccelerationMPerSecSquared);
        driveTemp.set(moduleData.driveTempCelcius);
        driveVolts.set(moduleData.driveAppliedVolts);
        driveCurrent.set(moduleData.driveCurrentAmps);

        turningSpeed.set(moduleData.turnVelocityRadPerSec);
        turningPosition.set(moduleData.turnPositionRad);
        turningTemp.set(moduleData.turnTempCelcius);
        turningVolts.set(moduleData.turnAppliedVolts);
        turningCurrent.set(moduleData.turnCurrentAmps);

    }
}
