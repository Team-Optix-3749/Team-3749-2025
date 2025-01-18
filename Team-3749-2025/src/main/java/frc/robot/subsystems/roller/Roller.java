package frc.robot.subsystems.roller;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.roller.RollerIO.RollerData;
import frc.robot.utils.ShuffleData;
import frc.robot.subsystems.roller.RollerConstants.RollerStates;

public abstract class Roller extends SubsystemBase {
    private RollerIO rollerIO;
    private RollerData rollerData;
    private RollerStates rollerStates;
    private final PIDController positionController;
    private double lastKnownPosition = 0.0;
    private RollerStates previousState = RollerConstants.RollerStates.STOP;

    private PIDController feedback = new PIDController(
            RollerConstants.kPSim,
            RollerConstants.kISim,
            RollerConstants.kDSim);

    private SimpleMotorFeedforward rollerFF = new SimpleMotorFeedforward(
            RollerConstants.kSSim,
            RollerConstants.kVSim,
            RollerConstants.kASim);

    private ShuffleData<Double> RollerVelocityLog = new ShuffleData<Double>(this.getName(), "roller velocity", 0.0);
    private ShuffleData<Double> RollerVoltageLog = new ShuffleData<Double>(this.getName(), "roller voltage", 0.0);
    private ShuffleData<Double> RollerCurrentLog = new ShuffleData<Double>(this.getName(), "roller current", 0.0);
    private ShuffleData<String> StateLog = new ShuffleData<String>(this.getName(), "state", RollerStates.STOP.name());
    private ShuffleData<Double> RollerPositionLog = new ShuffleData<Double>(this.getName(), "roller position", 0.0);
    private ShuffleData<Double> RollerLastKnownPositionLog = new ShuffleData<Double>(this.getName(), "roller last known position", 0.0);

    public Roller(RollerIO rollerIO) {
        this.rollerIO = rollerIO;
        this.positionController = new PIDController(15, 0, 10);
        this.rollerStates = RollerConstants.RollerStates.STOP;
        rollerData = new RollerData();
    }
    
    public void setVoltage(double volts) {
        rollerIO.setVoltage(volts);
    }

    public void setVelocity(double velocityRadPerSec) {
        double voltage = feedback.calculate(
            rollerData.rollerVelocityRadPerSec, 
            velocityRadPerSec) +
            rollerFF.calculate(velocityRadPerSec);

        setVoltage(voltage);
    }

    public RollerStates getState() {
        return rollerStates;
    }

    public void setState(RollerStates rollerStates) {
        this.rollerStates = rollerStates;
    }

    public void updateLastKnownPosition() {
        if (rollerStates == RollerConstants.RollerStates.MAINTAIN && previousState != RollerConstants.RollerStates.MAINTAIN) {
            lastKnownPosition = rollerData.rollerPositionRotations; 
        }
        previousState = rollerStates;
    }

    public void runRollerStates() {
        updateLastKnownPosition();
        
        switch(rollerStates) {
            case RUN:
                run();
                break;
            case MAINTAIN:
                maintain();
                break;
            case STOP:
                stop();
                break;
        }
    }

    public abstract void run();

    public void maintain() {
        double holdVoltage = positionController.calculate(
            rollerData.rollerPositionRotations, 
            lastKnownPosition
        );
        setVoltage(holdVoltage);
    }

    public void stop() {
        rollerIO.setVoltage(0.0);
    }

    @Override
    public void periodic() {
        rollerIO.updateData(rollerData);
        runRollerStates();

        RollerVelocityLog.set(rollerData.rollerVelocityRadPerSec);
        RollerVoltageLog.set(rollerData.rollerAppliedVolts);
        RollerCurrentLog.set(rollerData.currentAmps);
        RollerPositionLog.set(rollerData.rollerPositionRotations);
        RollerLastKnownPositionLog.set(lastKnownPosition);
        StateLog.set(rollerStates.name());
    }

}
