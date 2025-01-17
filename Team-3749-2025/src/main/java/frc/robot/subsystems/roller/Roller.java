package frc.robot.subsystems.roller;

import javax.sound.sampled.SourceDataLine;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.roller.RollerIO.RollerData;
import frc.robot.utils.ShuffleData;
import frc.robot.subsystems.roller.RollerConstants.RollerStates;

public abstract class Roller extends SubsystemBase {
    private RollerIO rollerIO;
    private RollerData rollerData;
    private RollerStates rollerStates;
    private final PIDController positionController;
    private double lastKnownVelocity = 0.0;

    private ShuffleData<Double> RollerVelocityLog = new ShuffleData<Double>(this.getName(), "roller velocity", 0.0);
    private ShuffleData<Double> RollerVoltageLog = new ShuffleData<Double>(this.getName(), "roller voltage", 0.0);
    private ShuffleData<Double> RollerCurrentLog = new ShuffleData<Double>(this.getName(), "roller current", 0.0);
    private ShuffleData<String> StateLog = new ShuffleData<String>(this.getName(), "state", RollerStates.STOP.name());

    public Roller(RollerIO rollerIO) {
        this.rollerIO = rollerIO;
        this.positionController = new PIDController(0.1, 0, 0);

        this.rollerStates = RollerConstants.RollerStates.RUN;
        rollerData = new RollerData();
    }
    
    public void setVoltage(double volts) {
        rollerIO.setVoltage(volts);
        rollerData.rollerVolts = volts;
    }

    public RollerStates getState() {
        return rollerStates;
    }

    public void setState(RollerStates rollerStates) {
        this.rollerStates = rollerStates;
    }

    public void runRollerStates() {
        switch(rollerStates) {
            case RUN:
                run();
                break;
            case MAINTAIN:
                lastKnownVelocity = rollerData.rollerVelocityRadPerSec;
                maintain();
                break;
            case STOP:
                stop();
                break;
        }
    }

    public abstract void run();

    public void maintain() {
        double holdVoltage = positionController.calculate(rollerData.rollerVelocityRadPerSec, lastKnownVelocity);
        rollerIO.setVoltage(holdVoltage);
    }

    public void stop() {
        rollerIO.setVoltage(0.0);
    }

    // public abstract void handleSpecialBehavior();

    @Override
    public void periodic() {
        rollerIO.updateData(rollerData);
        runRollerStates();

        RollerVelocityLog.set(rollerData.rollerVelocityRadPerSec);
        RollerVoltageLog.set(rollerData.rollerVolts);
        RollerCurrentLog.set(rollerData.currentAmps);
        StateLog.set(rollerStates.name());

        SmartDashboard.putNumber("Roller Velocity", rollerData.rollerVelocityRadPerSec);
        SmartDashboard.putNumber("Roller Voltage", rollerData.rollerVolts);
        SmartDashboard.putNumber("Roller Current", rollerData.currentAmps);
        SmartDashboard.putString("Roller State", rollerStates.name());
    }

}
