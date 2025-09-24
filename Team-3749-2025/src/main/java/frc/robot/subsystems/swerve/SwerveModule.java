package frc.robot.subsystems.swerve;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.*;
import frc.robot.subsystems.swerve.SwerveConstants.ControlConstants;
import frc.robot.subsystems.swerve.SwerveModuleIO.ModuleData;

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

    private SimpleMotorFeedforward drivingFeedFordward;
    private PIDController drivePID = new PIDController(ControlConstants.drivePID[0], ControlConstants.drivePID[1],
            ControlConstants.drivePID[2]);
    private PIDController turnPID = new PIDController(ControlConstants.turnPID[0], ControlConstants.turnPID[1],
            ControlConstants.turnPID[2]);

    private ModuleData moduleData = new ModuleData();
    private SwerveModuleIO moduleIO;

    private double previousSetpointVelocity = 0;
    private int index;

    public SwerveModule(int index, SwerveModuleIO SwerveModule) {
        this.index = index;
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

        drivingFeedFordward = new SimpleMotorFeedforward(ControlConstants.kSDriving.get(),
                ControlConstants.kVDriving.get(), ControlConstants.kADriving.get());
        turnPID.enableContinuousInput(0, 2.0 * Math.PI);
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
        double PID = drivePID.calculate(moduleData.driveVelocityMPerSec, speedMetersPerSecond);
        moduleIO.setDriveVoltage(PID + feedforward);
    }

    /**
     * 
     * @param positionRad - the angle setpoint (0-2pi) for the module
     */
    public void setTurnPosition(double positionRad) {
        double PID = turnPID.calculate(moduleData.turnPositionRad, positionRad);
        moduleIO.setTurnVoltage(PID);
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
        setDriveVoltage(0);
        setTurnVoltage(0);
    }

    public ModuleData getModuleData() {
        return moduleData;
    }

    // called within the swerve subsystem's periodic
    public void periodic() {
        moduleIO.updateData(moduleData);
        // // Logging

        Logger.recordOutput("Swerve/SwerveModule " + index + "/driveVelocity",
                moduleData.driveVelocityMPerSec);
        Logger.recordOutput("Swerve/SwerveModule " + index + "/drivePosition", moduleData.drivePositionM);
        Logger.recordOutput("Swerve/SwerveModule " + index + "/driveTemperature",
                moduleData.driveTempCelcius);
        Logger.recordOutput("Swerve/SwerveModule " + index + "/desiredVolts",
                moduleData.desiredVolts);
        Logger.recordOutput("Swerve/SwerveModule " + index + "/driveAppliedVolts",
                moduleData.driveAppliedVolts);
        Logger.recordOutput("Swerve/SwerveModule " + index + "/driveCurrentAmps",
                moduleData.driveCurrentAmps);

        Logger.recordOutput("Swerve/SwerveModule " + index + "/turnVelocity",
                moduleData.turnVelocityRadPerSec);
        Logger.recordOutput("Swerve/SwerveModule " + index + "/turnPosition", moduleData.turnPositionRad);
        Logger.recordOutput("Swerve/SwerveModule " + index + "/turnTemperature",
                moduleData.turnTempCelcius);
        Logger.recordOutput("Swerve/SwerveModule " + index + "/turnAppliedVolts",
                moduleData.turnAppliedVolts);
        Logger.recordOutput("Swerve/SwerveModule " + index + "/turnCurrentAmps",
                moduleData.turnCurrentAmps);
        Logger.recordOutput("Swerve/SwerveModule " + index + "/absoluteTurnPositionRad",
                moduleData.absoluteEncoderPositionRad);
        drivingFeedFordward = new SimpleMotorFeedforward(ControlConstants.kSDriving.get(),
                ControlConstants.kVDriving.get(), ControlConstants.kADriving.get());

    }
}
