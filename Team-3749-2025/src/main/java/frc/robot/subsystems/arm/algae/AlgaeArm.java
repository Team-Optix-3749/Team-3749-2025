package frc.robot.subsystems.arm.algae;

import frc.robot.Robot;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.real.ArmSparkMax;
import frc.robot.subsystems.arm.sim.ArmSim;
import frc.robot.utils.ShuffleData;
import frc.robot.utils.UtilityFunctions;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Subsystem class for the arm.
 * Handles control, logging, and state transitions for the Algae Arm subsystem.
 * 
 * @author Weston Gardner
 */

public class AlgaeArm extends Arm {

    // Arm I/O and state management

    private AlgaeConstants.ArmStates state = AlgaeConstants.ArmStates.STOPPED;

    private PIDController controller = new PIDController
    (
        AlgaeConstants.kP,
        AlgaeConstants.kI,
        AlgaeConstants.kD
    );

    private ShuffleData<String> stateLog = new ShuffleData<String>(this.getName(), "state", state.name());

    private Mechanism2d mechanism2d = new Mechanism2d(60, 60);
    private MechanismRoot2d armRoot = mechanism2d.getRoot("ArmRoot", 30, 30);
    private MechanismLigament2d armLigament = armRoot.append(new MechanismLigament2d("Algae Arm", 17, 0));

    /**
     * Constructor for the AlgaeArm subsystem.
     * Determines if simulation or real hardware is used.
     */

    public AlgaeArm() {
        if (Robot.isSimulation()) {

            armIO = new ArmSim(
                AlgaeConstants.numMotors,
                AlgaeConstants.armGearing,
                AlgaeConstants.momentOfInertia,
                AlgaeConstants.armLength_meters,
                AlgaeConstants.armMinAngle_degrees,
                AlgaeConstants.armMaxAngle_degrees,
                AlgaeConstants.simulateGravity,
                AlgaeConstants.armStartingAngle_degrees
            );

        } else {
            armIO = new ArmSparkMax(AlgaeConstants.motorId);
        }
        SmartDashboard.putData("Algae Arm Mechanism", mechanism2d);
    }

    /**
     * @return the current arm state.
     */
    public AlgaeConstants.ArmStates getState() {
        return state;
    }

    @Override
    public void stop() {
        setState(AlgaeConstants.ArmStates.STOPPED);
    }

    /**
     * @return whether the arm is in a stable state.
     */
    public boolean getIsStableState() {

        switch (state) {
            case STOWED:
                return data.positionUnits == AlgaeConstants.stowSetPoint_rad;
            case PROCESSOR:
                return data.positionUnits == AlgaeConstants.processorSetPoint_rad;
            case ALGAE_PICKUP:
                return data.positionUnits == AlgaeConstants.algaePickUpSetPoint_rad;
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
        this.state = (AlgaeConstants.ArmStates) state;
    }

    /**
     * Runs the logic for the current arm state.
     */
    private void runState() {
        switch (state) {
            case STOWED:
                setVoltage(controller.calculate(data.positionUnits, AlgaeConstants.stowSetPoint_rad) + calculateFeedForward());
                break;
            case PROCESSOR:
                setVoltage(controller.calculate(data.positionUnits, AlgaeConstants.processorSetPoint_rad) + calculateFeedForward());
                break;
            case ALGAE_PICKUP:
                setVoltage(controller.calculate(data.positionUnits, AlgaeConstants.algaePickUpSetPoint_rad) + calculateFeedForward());
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
        double feedForward = AlgaeConstants.kG * Math.cos(data.positionUnits);
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
