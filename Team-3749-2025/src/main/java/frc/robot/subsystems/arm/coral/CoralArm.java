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
import frc.robot.subsystems.arm.coral.CoralArmIO.ArmData;
import frc.robot.subsystems.arm.coral.real.CoralArmSparkMax;
import frc.robot.subsystems.arm.coral.sim.CoralArmSim;
import frc.robot.utils.ShuffleData;
import frc.robot.utils.UtilityFunctions;

/**
 * Subsystem class for the coral arm
 *
 * @author Weston Gardner
 */
public class CoralArm extends SubsystemBase{

	// Profiled PID Controller for arm movement, used to control position with velocity and acceleration constraints.
	private ProfiledPIDController controller = new ProfiledPIDController(
			CoralConstants.kP, // Proportional constant
			CoralConstants.kI, // Integral constant
			CoralConstants.kD, // Derivative constant
			new TrapezoidProfile.Constraints( // Constraints on velocity and acceleration
					CoralConstants.maxVelocity, 
					CoralConstants.maxAcceleration));

	// Arm feedforward to calculate the necessary voltage for the arm's movement.
	private ArmFeedforward feedforward = new ArmFeedforward(
			CoralConstants.kS, 
			CoralConstants.kG, 
			CoralConstants.kV, 
			CoralConstants.kA);

	// The I/O interface for controlling the arm's motors (either real hardware or simulated).
	private CoralArmIO armIO;
	// Stores the arm's current data (e.g., position, velocity, etc.).
	private ArmData data = new ArmData();
	// The current state of the arm (e.g., stopped, stowed).
	private CoralConstants.ArmStates state = CoralConstants.ArmStates.STOPPED;

	// Shuffleboard data for logging and displaying real-time data in the dashboard.
	private ShuffleData<String> currentCommandLog = new ShuffleData<>(this.getName(), "current command", "None");
	private ShuffleData<Double> positionUnitsLog = new ShuffleData<>(this.getName(), "position units", 0.0);
	private ShuffleData<Double> velocityUnitsLog = new ShuffleData<>(this.getName(), "velocity units", 0.0);
	private ShuffleData<Double> inputVoltsLog = new ShuffleData<>(this.getName(), "input volts", 0.0);
	private ShuffleData<Double> motorAppliedVoltsLog = new ShuffleData<>(this.getName(),
			"first motor applied volts", 0.0);
	private ShuffleData<Double> motorCurrentAmpsLog = new ShuffleData<>(this.getName(),
			"first motor current amps", 0.0);
	private ShuffleData<Double> motorTempCelciusLog = new ShuffleData<>(this.getName(),
			"first motor temp celcius", 0.0);
	private ShuffleData<String> stateLog = new ShuffleData<String>(this.getName(), "state", state.name());

	// For visualizing the arm mechanism in simulation using the SmartDashboard.
	private Mechanism2d mechanism2d = new Mechanism2d(60, 60);
	private MechanismRoot2d armRoot = mechanism2d.getRoot("ArmRoot", 30, 30);
	private MechanismLigament2d armLigament = armRoot.append(new MechanismLigament2d("Coral Arm", 24, 0));

	/**
	 * Constructor for the CoralArm subsystem. Determines if simulation or real
	 * hardware is used.
	 */
	public CoralArm() {

		// If the robot is in simulation, use the simulated I/O for the arm.
		if (Robot.isSimulation()) {
			armIO = new CoralArmSim();

		} else { 
			// If running on real hardware, use SparkMax motors for the arm.
			armIO = new CoralArmSparkMax(CoralConstants.motorID);
		}
		
		// Add the arm visualization to the SmartDashboard
		SmartDashboard.putData("Coral Arm Mechanism", mechanism2d);
	}
    
	// GET FUNCTIONS

	/**
	 * @return The current arm state (e.g., STOPPED, STOWED, etc.)
	 */
	public CoralConstants.ArmStates getState() {
		return state;
	}

	/**
	 * @return The current position of the arm in radians.
	 */
	public double getPositionRad() {
		return data.positionUnits; // Return the arm's current position.
	}

	/**
	 * @return Whether the arm is in a stable state. Checks if the arm is within a margin
	 * of error for its set positions.
	 */
	public boolean getIsStableState() {

		switch (state) {
			case STOWED:
				return UtilityFunctions.withinMargin(CoralConstants.stateMarginOfError, CoralConstants.stowSetPoint_rad, data.positionUnits);
			case HAND_OFF:
				return UtilityFunctions.withinMargin(CoralConstants.stateMarginOfError, CoralConstants.handOffSetPoint_rad, data.positionUnits);
			case CORAL_PICKUP:
				return UtilityFunctions.withinMargin(CoralConstants.stateMarginOfError, CoralConstants.coralPickUpSetPoint_rad, data.positionUnits);
			case STOPPED:
				return UtilityFunctions.withinMargin(CoralConstants.stateMarginOfError, 0, data.velocityUnits); // Ensure velocity is near zero when stopped.
			default:
				return false; // Return false if the state is unrecognized.
		}
	}


	// SET FUNCTIONS

	/**
	 * Sets the voltage to the arm motors. This directly controls the motor voltage.
	 * @param volts The voltage to apply to the arm motors.
	 */
	public void setVoltage(double volts) {
		armIO.setVoltage(volts);
	}

	/**
	 * Sets the state of the arm (e.g., STOPPED, STOWED, etc.). This will move the arm
	 * to preset angles or stop it depending on the state.
	 * @param state The new state for the arm.
	 */
	public void setState(CoralConstants.ArmStates state) {
		this.state = (CoralConstants.ArmStates) state;
		switch (this.state) {
			case STOPPED:
				stop(); // Stop the arm if in STOPPED state.
				break;
			case STOWED:
				setGoal(CoralConstants.stowSetPoint_rad); // Set the goal to the stowed position.
				break;
			case CORAL_PICKUP:
				setGoal(CoralConstants.coralPickUpSetPoint_rad); // Set the goal to the coral pickup position.
			case HAND_OFF:
				setGoal(CoralConstants.handOffSetPoint_rad); // Set the goal to the hand-off position.
			default:
				stop(); // Stop the arm in any unrecognized state.
				break;
		}
	}

	/**
	 * Sets the target position for the arm's PID controller.
	 * @param setPoint The desired target position for the arm in radians.
	 */
	public void setGoal(double setPoint) {
		controller.setGoal(setPoint); // Set the PID controller's goal.
	}

	// UTILITY FUNCTIONS

	/**
	 * Stops the arm completely. This method is for use in emergencies or on startup.
	 */
	public void stop() {
		setVoltage(0); // Apply zero volts to stop the arm.
	}

	/**
	 * Moves the arm to its goal using both PID control and feedforward calculations.
	 * This method combines PID and feedforward to control the arm's movement.
	 */
	private void moveToGoal() {
		// Get the setpoint from the PID controller
		State firstState = controller.getSetpoint();

		// Calculate the PID control voltage based on the arm's current position
		double pidVoltage = controller.calculate(getPositionRad());

		State nextState = controller.getSetpoint(); // Get the next state of the setpoint

		// Calculate the feedforward voltage based on velocity
		double ffVoltage = feedforward.calculate(firstState.velocity, nextState.velocity);

		// Apply the combined PID and feedforward voltages to the arm
		armIO.setVoltage(pidVoltage + ffVoltage);
	}



	// PERIODIC FUNCTIONS

	/** 
	 * Runs the logic for the current arm state. This is called periodically to update the arm's behavior.
	 */
	private void runState() {
		switch (state) {
			case STOPPED:
				stop(); // If the arm is stopped, we stop it.
				break;
			default:
				moveToGoal(); // In other states, move the arm to its goal position.
				break;
		}
	}

	/** 
	 * Logs the arm's data to Shuffleboard for monitoring. This is useful for debugging and analysis.
	 */
	private void logData() {
		// Log various arm parameters to Shuffleboard
		currentCommandLog.set(
				this.getCurrentCommand() == null ? "None" : this.getCurrentCommand().getName());
		positionUnitsLog.set(data.positionUnits);
		velocityUnitsLog.set(data.velocityUnits);
		inputVoltsLog.set(data.inputVolts);
		motorAppliedVoltsLog.set(data.motorAppliedVolts);
		motorCurrentAmpsLog.set(data.motorCurrentAmps);
		motorTempCelciusLog.set(data.motorTempCelcius);

		// Update the visualization on the SmartDashboard with the arm's position
		armLigament.setAngle(Math.toDegrees(data.positionUnits));

		stateLog.set(state.name());
	}

	/** 
	 * Periodic method called every loop to update the arm's behavior and log data.
	 */
	@Override
	public void periodic() {
		// Update the arm's data from the I/O interface
		armIO.updateData(data);

		// Run the state logic based on the current arm state
		runState();

		// Log the arm's data to Shuffleboard
		logData();
	}
}
