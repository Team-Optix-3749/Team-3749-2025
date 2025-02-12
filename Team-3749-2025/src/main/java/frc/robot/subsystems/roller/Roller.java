package frc.robot.subsystems.roller;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.roller.RollerIO.RollerData;
import frc.robot.subsystems.roller.real.RollerSparkMax;
import frc.robot.subsystems.roller.sim.RollerSim;
import frc.robot.utils.ShuffleData;
import frc.robot.Robot;
import frc.robot.subsystems.roller.RollerConstants.Implementations;
import frc.robot.subsystems.roller.RollerConstants.RollerStates;

/**
 * Roller subsystem
 *
 * @author Lilian Wu
 */
public abstract class Roller extends SubsystemBase {
    protected RollerData rollerData;
    protected RollerStates rollerState;
    
    private RollerIO rollerIO;
    private SimpleMotorFeedforward rollerFF;
    private double lastKnownPosition = 0.0;

    private ShuffleData<Double> rollerVelocityLog;
    private ShuffleData<Double> rollerVoltageLog;
    private ShuffleData<Double> rollerCurrentLog;
    private ShuffleData<String> stateLog;
    private ShuffleData<Double> rollerPositionLog;
    private ShuffleData<Double> rollerLastKnownPositionLog;

    public Roller(Implementations implementation, SimpleMotorFeedforward rollerFF) {
        rollerIO = Robot.isSimulation() ? new RollerSim(implementation)
                : new RollerSparkMax(implementation);

        String name = implementation.name();
        this.rollerFF = rollerFF;
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
        rollerIO.setVelocity(velocityRadPerSec, rollerFF.calculate(velocityRadPerSec));
    }

    public RollerStates getState() {
        return rollerState;
    }

    /**
     * Sets the roller state
     * 
     * Stores last position when the state is maintain
     */
    public void setState(RollerStates rollerState) {
        this.rollerState = rollerState;
        if (rollerState == RollerConstants.RollerStates.MAINTAIN) {
            lastKnownPosition = rollerData.rollerPositionRad;
        }
    }

    public void runRollerStates() {
        switch (rollerState) {
            case RUN:
                run();
                break;
            case MAINTAIN:
                maintain();
                break;
            case STOP:
                stop();
                break;
            case SCORE:
                score();
                break;
        }
    }

    public abstract void run();

    /**
     * Maintains the roller's last known position
     */
    public void maintain() {
        rollerIO.setPosition(rollerData.rollerPositionRad, lastKnownPosition);
    }

    public void stop() {
        rollerIO.setVoltage(0.0);
    }

    public abstract void score();
    
    @Override
    public void periodic() {
        rollerIO.updateData(rollerData);
        runRollerStates();

        rollerVelocityLog.set(rollerData.rollerVelocityRadPerSec);
        rollerVoltageLog.set(rollerData.rollerAppliedVolts);
        rollerCurrentLog.set(rollerData.currentAmps);
        rollerPositionLog.set(rollerData.rollerPositionRad);
        rollerLastKnownPositionLog.set(lastKnownPosition);
        stateLog.set(rollerState.name());
    }

}
