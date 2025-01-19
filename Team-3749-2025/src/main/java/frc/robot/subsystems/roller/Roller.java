package frc.robot.subsystems.roller;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.roller.RollerIO.RollerData;
import frc.robot.subsystems.roller.real.RollerSparkMax;
import frc.robot.subsystems.roller.sim.RollerSim;
import frc.robot.utils.ShuffleData;
import frc.robot.Robot;
import frc.robot.subsystems.roller.RollerConstants.Implementations;
import frc.robot.subsystems.roller.RollerConstants.RollerStates;

public abstract class Roller extends SubsystemBase {
    private RollerIO rollerIO;
    private RollerData rollerData;
    private RollerStates rollerState;
    private PIDController positionController;
    private PIDController velocityController;
    private SimpleMotorFeedforward rollerFF;
    private double lastKnownPosition = 0.0;

    private ShuffleData<Double> rollerVelocityLog;
    private ShuffleData<Double> rollerVoltageLog;
    private ShuffleData<Double> rollerCurrentLog;
    private ShuffleData<String> stateLog;
    private ShuffleData<Double> rollerPositionLog;
    private ShuffleData<Double> rollerLastKnownPositionLog;

    public Roller(Implementations implementation, PIDController velocityController, SimpleMotorFeedforward rollerFF, PIDController positionController) {
        switch(implementation) {
            case ALGAE:
                rollerIO = Robot.isSimulation() ? new RollerSim(implementation) : new RollerSparkMax(RollerConstants.Algae.motorId);
                break;
            case CORAL:
                rollerIO = Robot.isSimulation() ? new RollerSim(implementation) : new RollerSparkMax(RollerConstants.Coral.motorId);
                break;
            case SCORING:
                rollerIO = Robot.isSimulation() ? new RollerSim(implementation) : new RollerSparkMax(RollerConstants.Scoring.motorId);
                break;
        }
        
        String name = implementation.name();
        this.velocityController = velocityController;
        this.rollerFF = rollerFF;
        this.positionController = positionController;
        this.rollerState = RollerConstants.RollerStates.STOP;
        rollerData = new RollerData();

        rollerVelocityLog = new ShuffleData<>(getName(), name + " Velocity", 0.0);
        rollerVoltageLog = new ShuffleData<>(getName(), name + " Voltage", 0.0);
        rollerCurrentLog = new ShuffleData<>(getName(), name + " Current", 0.0);
        stateLog = new ShuffleData<>(getName(), "State", RollerStates.STOP.name());
        rollerPositionLog = new ShuffleData<>(getName(), "Position", 0.0);
        rollerLastKnownPositionLog = new ShuffleData<>(getName(), "Last Known Position", 0.0);
    }
    
    public RollerIO getRollerIO() {
        return rollerIO;
    }

    public void setVoltage(double volts) {
        rollerIO.setVoltage(volts);
    }

    public void setVelocity(double velocityRadPerSec) {
        double voltage = velocityController.calculate(
            rollerData.rollerVelocityRadPerSec, 
            velocityRadPerSec) +
            rollerFF.calculate(velocityRadPerSec);

        setVoltage(voltage);
    }

    public RollerStates getState() {
        return rollerState;
    }

    public void setState(RollerStates rollerState) {
        this.rollerState = rollerState;
        if (rollerState == RollerConstants.RollerStates.MAINTAIN) {
            lastKnownPosition = rollerData.rollerPositionRotations; 
        }
    }

    public void runRollerStates() {        
        switch(rollerState) {
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

        rollerVelocityLog.set(rollerData.rollerVelocityRadPerSec);
        rollerVoltageLog.set(rollerData.rollerAppliedVolts);
        rollerCurrentLog.set(rollerData.currentAmps);
        rollerPositionLog.set(rollerData.rollerPositionRotations);
        rollerLastKnownPositionLog.set(lastKnownPosition);
        stateLog.set(rollerState.name());
    }

}
