package frc.robot.subsystems.arm.coral;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.arm.ClimbArmIO;
import frc.robot.subsystems.arm.ClimbArmIO.ArmData;
import frc.robot.subsystems.arm.climb.ClimbArmSim;
import frc.robot.subsystems.arm.climb.ClimbArmSparkMax;
import frc.robot.utils.ShuffleData;
import frc.robot.utils.UtilityFunctions;

/**
 * Subsystem class for the coral arm
 *
 * @author Weston Gardner
 */
public class CoralArm extends SubsystemBase{

	private ProfiledPIDController controller = new ProfiledPIDController(
			CoralConstants.kP,
			CoralConstants.kI,
			CoralConstants.kD,
			new TrapezoidProfile.Constraints(
					CoralConstants.maxVelocity,
					CoralConstants.maxAcceleration));

	private ArmFeedforward feedforward = new ArmFeedforward(
			CoralConstants.kS,
			CoralConstants.kG,
			CoralConstants.kV,
			CoralConstants.kA);

	private ClimbArmIO armIO;
	private ArmData data = new ArmData();
	private CoralConstants.ArmStates state = CoralConstants.ArmStates.STOPPED;

	private ShuffleData<String> currentCommandLog = new ShuffleData<>(this.getName(), "current command", "None");
	private ShuffleData<Double> positionUnitsLog = new ShuffleData<>(this.getName(), "position units", 0.0);
	private ShuffleData<Double> velocityUnitsLog = new ShuffleData<>(this.getName(), "velocity units", 0.0);
	private ShuffleData<Double> inputVoltsLog = new ShuffleData<Double>(this.getName(), "input volts", 0.0);
	private ShuffleData<Double> firstMotorAppliedVoltsLog = new ShuffleData<>(this.getName(),
			"first motor applied volts", 0.0);
	private ShuffleData<Double> secondMotorAppliedVoltsLog = new ShuffleData<>(this.getName(),
			"second motor applied volts", 0.0);
	private ShuffleData<Double> firstMotorCurrentAmpsLog = new ShuffleData<>(this.getName(),
			"first motor current amps", 0.0);
	private ShuffleData<Double> secondMotorCurrentAmpsLog = new ShuffleData<>(this.getName(),
			"second motor current amps", 0.0);
	private ShuffleData<Double> firstMotorTempCelciusLog = new ShuffleData<>(this.getName(),
			"first motor temp celcius", 0.0);
	private ShuffleData<Double> secondMotorTempCelciusLog = new ShuffleData<>(this.getName(),
			"second motor temp celcius", 0.0);
	private ShuffleData<String> stateLog = new ShuffleData<String>(this.getName(), "state", state.name());


	private Mechanism2d mechanism2d = new Mechanism2d(60, 60);
	private MechanismRoot2d armRoot = mechanism2d.getRoot("ArmRoot", 30, 30);
	private MechanismLigament2d armLigament = armRoot.append(new MechanismLigament2d("Coral Arm", 24, 0));

	/**
	 * Constructor for the CoralArm subsystem. Determines if simulation or real
	 * hardware is used.
	 */
	public CoralArm() {

		if (Robot.isSimulation()) {

			armIO = new ClimbArmSim(
					CoralConstants.numMotors,
					CoralConstants.armGearing,
					CoralConstants.momentOfInertia,
					CoralConstants.armLength_meters,
					CoralConstants.armMinAngle_degrees,
					CoralConstants.armMaxAngle_degrees,
					CoralConstants.simulateGravity,
					CoralConstants.armStartingAngle_degrees);

		} else {
			armIO = new ClimbArmSparkMax(CoralConstants.firstMotorID, CoralConstants.secondMotorID);
		}
		SmartDashboard.putData("Coral Arm Mechanism", mechanism2d);
	}


	// SET FUNCTIONS


	// Method to set the voltage for the arm
	public void setVoltage(double volts) {
		armIO.setVoltage(volts);
	}

	/**
	 * Sets the current state of the arm.
	 *
	 * @param state The new state for the arm.
	 */
	public void setState(CoralConstants.ArmStates state) {
		this.state = (CoralConstants.ArmStates) state;
		switch (this.state) {
			case STOPPED:
				stop();
				break;
			case STOWED:
				setGoal(CoralConstants.stowSetPoint_rad);
				break;
			case CORAL_PICKUP:
				setGoal(CoralConstants.coralPickUpSetPoint_rad);
			case HAND_OFF:
				setGoal(CoralConstants.handOffSetPoint_rad);
			default:
				stop();
				break;
		}
	}

	public void setGoal(double setPoint) {
		controller.setGoal(setPoint);
	}


	// GET FUNCTIONS


	/**
	 * @return the current arm state.
	 */
	public CoralConstants.ArmStates getState() {
		return state;
	}


	/**
	 * @return the current arm position.
	 */
	public double getPositionRad() {
		return data.positionUnits;
	}

	/**
	 * @return whether the arm is in a stable state.
	 */
	public boolean getIsStableState() {

		switch (state) {
			case STOWED:
				return UtilityFunctions.withinMargin(0.001, CoralConstants.stowSetPoint_rad, data.positionUnits);
			case HAND_OFF:
				return UtilityFunctions.withinMargin(0.001, CoralConstants.handOffSetPoint_rad, data.positionUnits);
			case CORAL_PICKUP:
				return UtilityFunctions.withinMargin(0.001, CoralConstants.coralPickUpSetPoint_rad, data.positionUnits);
			case STOPPED:
				return UtilityFunctions.withinMargin(0.001, 0, data.velocityUnits);
			default:
				return false;
		}
	}


	// UTILITY FUNCTIONS

	/**
	 * stops the arm completely, for use in emergencies or on startup
	 */
	public void stop() {
		setVoltage(0);
	}

	/**
	 * Move the arm to the setpoint using the PID controller and feedforward.
	 * combines PID control and feedforward to move the arm to desired position.
	 */
	private void moveToGoal() {
		// Get setpoint from the PID controller
		State firstState = controller.getSetpoint();

		// Calculate PID voltage based on the current position
		double pidVoltage = controller.calculate(getPositionRad());

		State nextState = controller.getSetpoint();

		// Calculate feedforward voltage
		double ffVoltage = feedforward.calculate(firstState.velocity, nextState.velocity);

		// Set the voltage for the arm motor (combine PID and feedforward)
		armIO.setVoltage(pidVoltage + ffVoltage);
	}



	// PERIODIC FUNCTIONS



	/** Runs the logic for the current arm state. */
	private void runState() {
		switch (state) {
			case STOPPED:
				stop();
				break;
			default:
				moveToGoal();
				break;
		}
	}

	/** Logs data to Shuffleboard. */
	private void logData() {
		currentCommandLog.set(
				this.getCurrentCommand() == null ? "None" : this.getCurrentCommand().getName());
		positionUnitsLog.set(data.positionUnits);
		velocityUnitsLog.set(data.velocityUnits);
		inputVoltsLog.set(data.inputVolts);
		firstMotorAppliedVoltsLog.set(data.firstMotorAppliedVolts);
		secondMotorAppliedVoltsLog.set(data.secondMotorAppliedVolts);
		firstMotorCurrentAmpsLog.set(data.firstMotorCurrentAmps);
		secondMotorCurrentAmpsLog.set(data.secondMotorCurrentAmps);
		firstMotorTempCelciusLog.set(data.firstMotorTempCelcius);
		secondMotorTempCelciusLog.set(data.secondMotorTempCelcius);

		armLigament.setAngle(Math.toDegrees(data.positionUnits));

		stateLog.set(state.name());
	}

	/** Periodic method for updating arm behavior. */
	@Override
	public void periodic() {

		armIO.updateData(data);

		runState();

		logData();
	}
}
