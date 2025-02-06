package frc.robot.subsystems.arm.climb;

import frc.robot.Robot;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.sim.ArmSim;
import frc.robot.utils.ShuffleData;
import frc.robot.utils.SysIdTuner;
import frc.robot.utils.UtilityFunctions;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.arm.climb.ClimbArmIO.ArmData;
import frc.robot.utils.LoggedTunableNumber;
import frc.robot.utils.ShuffleData;
import frc.robot.utils.UtilityFunctions;

/**
 * Subsystem class for the climb arm
 *
 * @author Weston Gardner
 */

public class ClimbArm extends SubsystemBase {

    private ClimbConstants.ArmStates state = ClimbConstants.ArmStates.STOPPED;

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
	private LoggedTunableNumber positionUnitsLog = new LoggedTunableNumber(this.getName() + "/position units", 0.0);
	private LoggedTunableNumber velocityUnitsLog = new LoggedTunableNumber(this.getName() + "/velocity units", 0.0);
	private LoggedTunableNumber inputVoltsLog = new LoggedTunableNumber(this.getName() + "/input volts", 0.0);
	private LoggedTunableNumber frontMotorAppliedVoltsLog = new LoggedTunableNumber(this.getName() +
			"/first motor applied volts", 0.0);
	private LoggedTunableNumber backMotorAppliedVoltsLog = new LoggedTunableNumber(this.getName() +
			"/second motor applied volts", 0.0);
	private LoggedTunableNumber frontMotorCurrentAmpsLog = new LoggedTunableNumber(this.getName() +
			"/first motor current amps", 0.0);
	private LoggedTunableNumber backMotorCurrentAmpsLog = new LoggedTunableNumber(this.getName() +
			"/second motor current amps", 0.0);
	private LoggedTunableNumber frontMotorTempCelciusLog = new LoggedTunableNumber(this.getName() +
			"/first motor temp celcius", 0.0);
	private LoggedTunableNumber backMotorTempCelciusLog = new LoggedTunableNumber(this.getName() +
			"/second motor temp celcius", 0.0);
	private ShuffleData<String> stateLog = new ShuffleData<String>(this.getName(), "state", state.name());

	private Mechanism2d mechanism2d = new Mechanism2d(60, 60);
	private MechanismRoot2d armRoot = mechanism2d.getRoot("ArmRoot", 30, 30);
	private MechanismLigament2d armLigament = armRoot.append(new MechanismLigament2d("Climb Arm", 24, 0));

    private SysIdTuner sysIdTuner;

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

        sysIdTuner = new SysIdTuner("climb arm", getConfig(), this, armIO::setVoltage, getMotorData());
    }

    public SysIdTuner getSysIdTuner(){
        System.out.println(sysIdTuner);
        return sysIdTuner;
    }

	/**
	 * @return the current arm state.
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
				return UtilityFunctions.withinMargin(0.001, ClimbConstants.stowSetPoint_rad, data.positionUnits);
			case PREPARE_FOR_CLIMB:
				return UtilityFunctions.withinMargin(0.001, ClimbConstants.PrepareForClimbSetPoint_rad, data.positionUnits);
			case CLIMB:
				return UtilityFunctions.withinMargin(0.001, ClimbConstants.climbSetPoint_rad, data.positionUnits);
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
