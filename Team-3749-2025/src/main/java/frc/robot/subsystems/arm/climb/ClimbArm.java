package frc.robot.subsystems.arm.climb;

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
import frc.robot.subsystems.arm.climb.ClimbArmIO.ArmData;
import frc.robot.subsystems.arm.climb.real.ClimbArmSparkMax;
import frc.robot.subsystems.arm.climb.sim.ClimbArmSim;
import frc.robot.utils.ShuffleData;
import frc.robot.utils.UtilityFunctions;

/**
 * Subsystem class for the climb arm
 *
 * @author Weston Gardner
 */
public class ClimbArm extends SubsystemBase {

	private ProfiledPIDController controller = new ProfiledPIDController(
			ClimbConstants.kP,
			ClimbConstants.kI,
			ClimbConstants.kD,
			new TrapezoidProfile.Constraints(
					ClimbConstants.maxVelocity,
					ClimbConstants.maxAcceleration));

	private ArmFeedforward feedforward = new ArmFeedforward(
			ClimbConstants.kS,
			ClimbConstants.kG,
			ClimbConstants.kV,
			ClimbConstants.kA);

	private ClimbArmIO armIO;
	private ArmData data = new ArmData();
	private ClimbConstants.ArmStates state = ClimbConstants.ArmStates.STOPPED;

	private ShuffleData<String> currentCommandLog = new ShuffleData<>(this.getName(), "current command", "None");
	private ShuffleData<Double> positionUnitsLog = new ShuffleData<>(this.getName(), "position units", 0.0);
	private ShuffleData<Double> velocityUnitsLog = new ShuffleData<>(this.getName(), "velocity units", 0.0);
	private ShuffleData<Double> inputVoltsLog = new ShuffleData<Double>(this.getName(), "input volts", 0.0);
	private ShuffleData<Double> frontMotorAppliedVoltsLog = new ShuffleData<>(this.getName(),
			"first motor applied volts", 0.0);
	private ShuffleData<Double> backMotorAppliedVoltsLog = new ShuffleData<>(this.getName(),
			"second motor applied volts", 0.0);
	private ShuffleData<Double> frontMotorCurrentAmpsLog = new ShuffleData<>(this.getName(),
			"first motor current amps", 0.0);
	private ShuffleData<Double> backMotorCurrentAmpsLog = new ShuffleData<>(this.getName(),
			"second motor current amps", 0.0);
	private ShuffleData<Double> frontMotorTempCelciusLog = new ShuffleData<>(this.getName(),
			"first motor temp celcius", 0.0);
	private ShuffleData<Double> backMotorTempCelciusLog = new ShuffleData<>(this.getName(),
			"second motor temp celcius", 0.0);
	private ShuffleData<String> stateLog = new ShuffleData<String>(this.getName(), "state", state.name());

	private Mechanism2d mechanism2d = new Mechanism2d(60, 60);
	private MechanismRoot2d armRoot = mechanism2d.getRoot("ArmRoot", 30, 30);
	private MechanismLigament2d armLigament = armRoot.append(new MechanismLigament2d("Climb Arm", 24, 0));

	/**
	 * Constructor for the CoralArm subsystem. Determines if simulation or real
	 * hardware is used.
	 */
	public ClimbArm() {

		if (Robot.isSimulation()) {

			armIO = new ClimbArmSim();

		} else {
			armIO = new ClimbArmSparkMax(ClimbConstants.frontMotorId, ClimbConstants.backMotorId);
		}
		SmartDashboard.putData("Climb Arm Mechanism", mechanism2d);
	}
	
	// GET FUNCTIONS

	/**
	 * @return the state the arm is in
	 */
	public ClimbConstants.ArmStates getState() {
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
				return UtilityFunctions.withinMargin(ClimbConstants.stateMarginOfError, ClimbConstants.stowSetPoint_rad, data.positionUnits);
			case PREPARE_FOR_CLIMB:
				return UtilityFunctions.withinMargin(ClimbConstants.stateMarginOfError, ClimbConstants.PrepareForClimbSetPoint_rad,
						data.positionUnits);
			case CLIMB:
				return UtilityFunctions.withinMargin(ClimbConstants.stateMarginOfError, ClimbConstants.climbSetPoint_rad, data.positionUnits);
			case STOPPED:
				return UtilityFunctions.withinMargin(ClimbConstants.stateMarginOfError, 0, data.velocityUnits);
			default:
				return false;
		}
	}


	// SET FUNCTIONS

	/**
	 * method to set the voltage for the arm
	 * 
	 * @param volts
	 */
	public void setVoltage(double volts) {
		armIO.setVoltage(volts);
	}

	/**
	 * Sets the current state of the arm.
	 *
	 * @param state The new state for the arm.
	 */
	public void setState(ClimbConstants.ArmStates state) {
		this.state = (ClimbConstants.ArmStates) state;
		switch (this.state) {
			case STOPPED:
				stop();
				break;
			case STOWED:
				setGoal(ClimbConstants.stowSetPoint_rad);
				break;
			case PREPARE_FOR_CLIMB:
				setGoal(ClimbConstants.PrepareForClimbSetPoint_rad);
			case CLIMB:
				setGoal(ClimbConstants.climbSetPoint_rad);
			default:
				stop();
				break;
		}
	}

	/**
	 * method to set the goal of the controller
	 * 
	 * @param setPoint
	 */
	public void setGoal(double setPoint) {
		controller.setGoal(setPoint);
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

	/**
	 * Runs the logic for the current arm state.
	 */
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

	/**
	 * Logs data to Shuffleboard.
	 */
	private void logData() {
		currentCommandLog.set(
				this.getCurrentCommand() == null ? "None" : this.getCurrentCommand().getName());
		positionUnitsLog.set(data.positionUnits);
		velocityUnitsLog.set(data.velocityUnits);
		inputVoltsLog.set(data.inputVolts);
		frontMotorAppliedVoltsLog.set(data.frontMotorAppliedVolts);
		backMotorAppliedVoltsLog.set(data.backMotorAppliedVolts);
		frontMotorCurrentAmpsLog.set(data.frontMotorCurrentAmps);
		backMotorCurrentAmpsLog.set(data.backMotorCurrentAmps);
		frontMotorTempCelciusLog.set(data.frontMotorTempCelcius);
		backMotorTempCelciusLog.set(data.backMotorTempCelcius);

		armLigament.setAngle(Math.toDegrees(data.positionUnits));

		stateLog.set(state.name());
	}

	/**
	 * Periodic method for updating arm behavior.
	 */
	@Override
	public void periodic() {

		armIO.updateData(data);

		runState();

		logData();
	}
}
