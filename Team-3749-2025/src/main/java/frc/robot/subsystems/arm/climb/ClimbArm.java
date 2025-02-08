package frc.robot.subsystems.arm.climb;

import frc.robot.Robot;
import frc.robot.utils.ShuffleData;
import frc.robot.utils.SysIdTuner;
import frc.robot.utils.UtilityFunctions;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.arm.climb.ClimbArmIO.ArmData;
import frc.robot.utils.LoggedTunableNumber;
import frc.robot.utils.MotorData;
import static edu.wpi.first.units.Units.*;

import java.util.Map;
import frc.robot.subsystems.arm.climb.real.ClimbArmSparkMax;
import frc.robot.subsystems.arm.climb.sim.ClimbArmSim;
import frc.robot.utils.ShuffleData;
import frc.robot.utils.UtilityFunctions;

import static edu.wpi.first.units.Units.*;

/**
 * Subsystem class for the climb arm
 *
 * @author Weston Gardner
 */

public class ClimbArm extends SubsystemBase {

	private ProfiledPIDController profile = new ProfiledPIDController(
			0, 0, 0,
			new TrapezoidProfile.Constraints(
					ClimbArmConstants.maxVelocity,
					ClimbArmConstants.maxAcceleration));

	private ArmFeedforward feedforward = new ArmFeedforward(
			ClimbArmConstants.kS,
			ClimbArmConstants.kG,
			ClimbArmConstants.kV,
			ClimbArmConstants.kA);

	private ClimbArmIO armIO;
	private ArmData data = new ArmData();
	private ClimbArmConstants.ArmStates state = ClimbArmConstants.ArmStates.STOPPED;

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

	private LoggedTunableNumber kG = new LoggedTunableNumber(this.getName() + "/kG", ClimbArmConstants.kG);
	private LoggedTunableNumber kP = new LoggedTunableNumber(this.getName() + "/kP", ClimbArmConstants.kP);
	private LoggedTunableNumber kI = new LoggedTunableNumber(this.getName() + "/kI", ClimbArmConstants.kI);
	private LoggedTunableNumber kD = new LoggedTunableNumber(this.getName() + "/kD", ClimbArmConstants.kD);
	private LoggedTunableNumber kS = new LoggedTunableNumber(this.getName() + "/kS", ClimbArmConstants.kS);
	private LoggedTunableNumber kV = new LoggedTunableNumber(this.getName() + "/kV", ClimbArmConstants.kV);
	private LoggedTunableNumber kA = new LoggedTunableNumber(this.getName() + "/kA", ClimbArmConstants.kA);
	private LoggedTunableNumber maxVelocity = new LoggedTunableNumber(this.getName() + "/max velocity",
			ClimbArmConstants.maxVelocity);
	private LoggedTunableNumber maxAcceleration = new LoggedTunableNumber(this.getName() + "/max acceleration",
			ClimbArmConstants.maxAcceleration);
    StructPublisher<Pose3d> publisher = NetworkTableInstance.getDefault()
            .getStructTopic("ClimbArm Pose", Pose3d.struct).publish();
	/**
	 * Constructor for the CoralArm subsystem. Determines if simulation or real
	 * hardware is used.
	 */

	private SysIdTuner sysIdTuner;

	SysIdRoutine.Config config = new SysIdRoutine.Config(
			Volts.per(Seconds).of(1), // Voltage ramp rate
			Volts.of(4), // Max voltage
			Seconds.of(4) // Test duration
	);

	Map<String, MotorData> motorData = Map.of(
			"arm_motor", new MotorData(
					data.appliedVolts,
					data.positionUnits,
					data.velocityUnits,
					data.accelerationUnits));

	public ClimbArm() {
		if (Robot.isSimulation()) {

			armIO = new ClimbArmSim();

		} else {
			armIO = new ClimbArmSparkMax();
		}
		SmartDashboard.putData("Climb Arm Mechanism", mechanism2d);

		sysIdTuner = new SysIdTuner("climb arm", getConfig(), this, armIO::setVoltage, getMotorData());
	}

	public Map<String, MotorData> getMotorData() {
		return motorData;
	}

	public SysIdRoutine.Config getConfig() {
		return config;
	}

	public SysIdTuner getSysIdTuner() {
		System.out.println(sysIdTuner);
		return sysIdTuner;
	}

	// GET FUNCTIONS

	/**
	 * @return the current arm state.
	 */
	public ClimbArmConstants.ArmStates getState() {
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
				return UtilityFunctions.withinMargin(ClimbArmConstants.stateMarginOfError,
						ClimbArmConstants.stowSetPoint_rad, data.positionUnits);
			case PREPARE_FOR_CLIMB:
				return UtilityFunctions.withinMargin(ClimbArmConstants.stateMarginOfError,
						ClimbArmConstants.PrepareForClimbSetPoint_rad,
						data.positionUnits);
			case CLIMB:
				return UtilityFunctions.withinMargin(ClimbArmConstants.stateMarginOfError,
						ClimbArmConstants.climbSetPoint_rad, data.positionUnits);
			case STOPPED:
				return UtilityFunctions.withinMargin(ClimbArmConstants.stateMarginOfError, 0, data.velocityUnits);
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
	public void setState(ClimbArmConstants.ArmStates state) {
		this.state = (ClimbArmConstants.ArmStates) state;
		switch (this.state) {
			case STOPPED:
				stop();
				break;
			case STOWED:
				setGoal(ClimbArmConstants.stowSetPoint_rad);
				break;
			case PREPARE_FOR_CLIMB:
				setGoal(ClimbArmConstants.PrepareForClimbSetPoint_rad);
			case CLIMB:
				setGoal(ClimbArmConstants.climbSetPoint_rad);
			default:
				stop();
				break;
		}
	}

      

    private Angle getPitch() {
        return Angle.ofBaseUnits(-data.positionUnits + Units.degreesToRadians(0), Radians); // remove offset once climb
                                                                                            // arm code is fixed
    }

    private Pose3d getPose3d() {
        //
        Pose3d pose = new Pose3d(0, 0.18, 0.165,
                new Rotation3d(getPitch(), Angle.ofBaseUnits(0, Radians), Angle.ofBaseUnits(0, Radians)));
        return pose;
    }
	/**
	 * method to set the goal of the controller
	 * 
	 * @param setPoint
	 */
	public void setGoal(double setPoint) {
		profile.setGoal(setPoint);
	}

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
		State firstState = profile.getSetpoint();

		// Calculate PID voltage based on the current position
		profile.calculate(getPositionRad());

		State nextState = profile.getSetpoint();

		// Calculate feedforward voltage
		double ffVoltage = feedforward.calculate(firstState.velocity, nextState.velocity);

		// Set the voltage for the arm motor (combine PID and feedforward)
		armIO.setPosition(firstState.position, ffVoltage);
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
		frontMotorAppliedVoltsLog.set(data.frontMotorAppliedVolts);
		backMotorAppliedVoltsLog.set(data.backMotorAppliedVolts);
		frontMotorCurrentAmpsLog.set(data.frontMotorCurrentAmps);
		backMotorCurrentAmpsLog.set(data.backMotorCurrentAmps);
		frontMotorTempCelciusLog.set(data.frontMotorTempCelcius);
		backMotorTempCelciusLog.set(data.backMotorTempCelcius);

		armLigament.setAngle(Math.toDegrees(data.positionUnits));

		stateLog.set(state.name());

		ClimbArmConstants.kG = kG.get();
		ClimbArmConstants.kP = kP.get();
		ClimbArmConstants.kI = kI.get();
		ClimbArmConstants.kD = kD.get();
		ClimbArmConstants.kS = kS.get();
		ClimbArmConstants.kV = kV.get();
		ClimbArmConstants.kA = kA.get();
		ClimbArmConstants.maxVelocity = maxVelocity.get();
		ClimbArmConstants.maxAcceleration = maxAcceleration.get();
        publisher.set(getPose3d());
	}

	/** Periodic method for updating arm behavior. */
	@Override
	public void periodic() {

		armIO.updateData(data);

		runState();

		logData();

		motorData.get("arm_motor").position = data.positionUnits;
		motorData.get("arm_motor").acceleration = data.accelerationUnits;
		motorData.get("arm_motor").velocity = data.velocityUnits;
		motorData.get("arm_motor").appliedVolts = data.appliedVolts;
	}
}
