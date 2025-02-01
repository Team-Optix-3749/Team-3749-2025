package frc.robot.subsystems.arm.coral;

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
 * Subsystem class for the arm
 * 
 * @author Weston Gardner
 */

public class CoralArm extends Arm {

    private CoralArmConstants.ArmStates state = CoralArmConstants.ArmStates.STOPPED;

    private PIDController controller = new PIDController(
            CoralArmConstants.kP,
            CoralArmConstants.kI,
            CoralArmConstants.kD);

    private ShuffleData<String> stateLog = new ShuffleData<String>(this.getName(), "state", state.name());

    private Mechanism2d mechanism2d = new Mechanism2d(60, 60);
    private MechanismRoot2d armRoot = mechanism2d.getRoot("ArmRoot", 30, 30);
    private MechanismLigament2d armLigament = armRoot.append(new MechanismLigament2d("Coral Arm", 24, 0));


    /**
     * Constructor for the CoralArm subsystem.
     * Determines if simulation or real hardware is used.
     */
    public CoralArm() {
        if (Robot.isSimulation()) {

            armIO = new ArmSim(
                CoralArmConstants.numMotors,
                CoralArmConstants.armGearing,
                CoralArmConstants.momentOfInertia,
                CoralArmConstants.armLength_meters,
                CoralArmConstants.armMinAngle_degrees,
                CoralArmConstants.armMaxAngle_degrees,
                CoralArmConstants.simulateGravity,
                CoralArmConstants.armStartingAngle_degrees);

        } else {
            armIO = new ArmSparkMax(CoralArmConstants.motorId);
        }
        SmartDashboard.putData("Coral Arm Mechanism", mechanism2d);
    }

    /**
     * @return the current arm state.
     */
    public CoralArmConstants.ArmStates getState() {
        return state;
    }

    @Override
    public void stop() {
        setState(CoralArmConstants.ArmStates.STOPPED);
    }

    /**
     * @return whether the arm is in a stable state.
     */
    public boolean getIsStableState() {

        switch (state) {
            case STOWED:
                return UtilityFunctions.withinMargin(0.01, CoralArmConstants.stowSetPoint_rad,
                data.positionUnits);
            case HAND_OFF:
                return UtilityFunctions.withinMargin(0.01, CoralArmConstants.handOffSetPoint_rad,
                data.positionUnits);
            case CORAL_PICKUP:
                return UtilityFunctions.withinMargin(0.01, CoralArmConstants.coralPickUpSetPoint_rad,
                data.positionUnits);
            case L1:
                return UtilityFunctions.withinMargin(0.01, CoralArmConstants.L1SetPoint_rad,
                data.positionUnits);
            case MOVING_DOWN:
                return data.velocityUnits < 0;
            case MOVING_UP:
                return data.velocityUnits > 0;
            case STOPPED:
                return UtilityFunctions.withinMargin(0.01, 0, data.velocityUnits);
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
        this.state = (CoralArmConstants.ArmStates) state;
    }

    /**
     * Runs the logic for the current arm state.
     */
    private void runState() {
        switch (state) {
            case STOWED:
                setVoltage(controller.calculate(data.positionUnits, CoralArmConstants.stowSetPoint_rad) + calculateFeedForward());
                break;
            case HAND_OFF:
                setVoltage(controller.calculate(data.positionUnits, CoralArmConstants.handOffSetPoint_rad) + calculateFeedForward());
                break;
            case CORAL_PICKUP:
                setVoltage(controller.calculate(data.positionUnits, CoralArmConstants.coralPickUpSetPoint_rad) + calculateFeedForward());
                break;
            case L1:
                setVoltage(controller.calculate(data.positionUnits, CoralArmConstants.L1SetPoint_rad) + calculateFeedForward());
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

    public boolean hasPiece(){
        return false;
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
        double feedForward = CoralArmConstants.kG * Math.cos(data.positionUnits);
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
