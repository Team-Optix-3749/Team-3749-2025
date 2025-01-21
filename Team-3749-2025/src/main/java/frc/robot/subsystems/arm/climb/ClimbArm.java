package frc.robot.subsystems.arm.climb;

import frc.robot.Robot;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.sim.ArmSim;
import frc.robot.utils.ShuffleData;
import frc.robot.utils.UtilityFunctions;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Subsystem class for the arm
 * 
 * @author Weston Gardner
 */

public class ClimbArm extends Arm {

    private ClimbConstants.ArmStates state = ClimbConstants.ArmStates.STOPPED;

    private PIDController controller = new PIDController(
        ClimbConstants.kP, 
        ClimbConstants.kI, 
        ClimbConstants.kD
    );

    private ShuffleData<String> stateLog = new ShuffleData<String>(this.getName(), "state", state.name());

    private Mechanism2d mechanism2d = new Mechanism2d(60, 60);
    private MechanismRoot2d armRoot = mechanism2d.getRoot("ArmRoot", 30, 30);
    private MechanismLigament2d armLigament = armRoot.append(new MechanismLigament2d("Climb Arm", 24, 0));


    public ClimbArm() {
        if (Robot.isSimulation()) {
            
            armIO = new ArmSim(
                ClimbConstants.numMotors, 
                ClimbConstants.armGearing, 
                ClimbConstants.momentOfInertia, 
                ClimbConstants.armLength_meters, 
                ClimbConstants.armMinAngle_degrees, 
                ClimbConstants.armMaxAngle_degrees, 
                ClimbConstants.simulateGravity, 
                ClimbConstants.armStartingAngle_degrees
            );

        } else {
            armIO = new ClimbSparkMax(ClimbConstants.firstMotorId, ClimbConstants.secondMotorId);
        }
        SmartDashboard.putData("Climb Arm Mechanism", mechanism2d);
    }


    /**
     * @return the current arm state.
     */
    public ClimbConstants.ArmStates getState() {
        return state;
    }

    @Override
    public void stop() {
        setState(ClimbConstants.ArmStates.STOPPED);
    }

    /**
     * @return whether the arm is in a stable state.
     */
    public boolean getIsStableState() {

        switch (state) {
            case STOWED:
                return data.positionUnits == ClimbConstants.stowSetPoint_rad;
            case PREPARE_FOR_CLIMB:
                return data.positionUnits == ClimbConstants.PrepareForClimbSetPoint_rad;
            case CLIMB:
                return data.positionUnits == ClimbConstants.climbSetPoint_rad;
            case MOVING_DOWN:
                return data.velocityUnits < 0;
            case MOVING_UP:
                return data.velocityUnits > 0;
            case STOPPED:
                return UtilityFunctions.withinMargin(0.001, 0, data.velocityUnits);
            default:
                return false;
        }
    }

    /**
     * Sets the current state of the arm.
     * 
     * @param state The new state for the arm.
     */
    @Override
    public void setState(Enum<?> state) {
        this.state = (ClimbConstants.ArmStates) state;
    }

    /**
     * Runs the logic for the current arm state.
     */
    private void runState() {
        switch (state) {
            case STOWED:
                setVoltage(controller.calculate(data.positionUnits, ClimbConstants.stowSetPoint_rad) + calculateFeedForward());
                break;
            case PREPARE_FOR_CLIMB:
                setVoltage(controller.calculate(data.positionUnits, ClimbConstants.PrepareForClimbSetPoint_rad) + calculateFeedForward());
                break;
            case CLIMB:
                setVoltage(controller.calculate(data.positionUnits, ClimbConstants.climbSetPoint_rad) + calculateFeedForward());
                break;
            case STOPPED:
                setVoltage(0 + calculateFeedForward());
                break;
            case MOVING_DOWN:
                setVoltage(-1 + calculateFeedForward());
                break;
            case MOVING_UP:
                setVoltage(1 + calculateFeedForward());
                break;
            default:
                break;
        }
    }

    /**
     * Logs data to Shuffleboard.
     */
    private void logData() {
        currentCommandLog.set(this.getCurrentCommand() == null ? "None" : this.getCurrentCommand().getName());
        positionUnitsLog.set(data.positionUnits);
        velocityUnitsLog.set(data.velocityUnits);
        inputVoltsLog.set(data.inputVolts);
        appliedVoltsLog.set(data.appliedVolts);
        currentAmpsLog.set(data.currentAmps);
        tempCelciusLog.set(data.tempCelcius);

        armLigament.setAngle(Math.toDegrees(data.positionUnits));

        stateLog.set(state.name());
    }

    private double calculateFeedForward() {
        double feedForward = ClimbConstants.kG * Math.cos(data.positionUnits);
        return feedForward;
    }

    /**
     * Periodic method for updating arm behavior.
     */
    @Override
    public void periodic() {

        armIO.updateData(data);

        logData();

        runState();
    }
}
