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

    public Roller(RollerIO rollerIO) {
        this.rollerIO = rollerIO;
        this.positionController = new PIDController(0.1, 0, 0);
        this.rollerStates = RollerConstants.RollerStates.RUN;
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

    public void runRollerStates() {
        switch(rollerStates) {
            case RUN:
                run();
                break;
            case MAINTAIN:
                lastKnownPosition = rollerData.rollerPositionRotations;
                maintain();
                break;
            case STOP:
                stop();
                break;
        }
    }

    public abstract void run();

    public void maintain() {
        double holdVoltage = positionController.calculate(rollerData.rollerPositionRotations, lastKnownPosition);
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
        RollerVoltageLog.set(rollerData.rollerAppliedVolts);
        RollerCurrentLog.set(rollerData.currentAmps);
        StateLog.set(rollerStates.name());
    }

}
