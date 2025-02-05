package frc.robot.subsystems.arm.climb.sim;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.subsystems.arm.climb.ClimbArmIO;
import frc.robot.subsystems.arm.climb.ClimbConstants;
import frc.robot.subsystems.arm.climb.ClimbArmIO.ArmData;
import frc.robot.utils.MiscConstants.SimConstants;

/**
 * IO implementation for an arm subsystem's simulation
 *
 * @author Weston Gardner
 */
public class ClimbArmSim implements ClimbArmIO {

	private SingleJointedArmSim armSim;

	/**
	 * creates a new arm simulation motor form the single jointed arm class
	 * 
	 * @param numMotors
	 * @param gearing
	 * @param momentOfInertia
	 * @param length_meters
	 * @param minAngle_degrees
	 * @param maxAngle_degrees
	 * @param simulateGravity
	 * @param startingAngle_Degrees
	 */
	public ClimbArmSim() {

		System.out.println("[Init] Creating ArmSim");

		armSim = new SingleJointedArmSim(
				DCMotor.getNEO(ClimbConstants.numMotors),
				ClimbConstants.armGearing,
				ClimbConstants.momentOfInertia,
				ClimbConstants.armLength_meters,
				ClimbConstants.armMinAngle_degrees * Math.PI / 180,
				ClimbConstants.armMaxAngle_degrees * Math.PI / 180,
				ClimbConstants.simulateGravity,
				ClimbConstants.armStartingAngle_degrees * Math.PI / 180);
	}

	private double inputVolts = 0;
	private double previousVelocity = 0;
	private double velocity = 0;

	/**
	 * Updates the set of loggable inputs for the sim.
	 *
	 * @param data
	 */
	@Override
	public void updateData(ArmData data) {
		armSim.update(0.02);
		previousVelocity = velocity;
		velocity = armSim.getVelocityRadPerSec();
		data.positionUnits = armSim.getAngleRads();
		data.velocityUnits = velocity;
		data.accelerationUnits = (velocity - previousVelocity) / SimConstants.loopPeriodSec;

		data.inputVolts = inputVolts;
		data.frontMotorAppliedVolts = inputVolts;
		data.backMotorAppliedVolts = inputVolts;
		data.frontMotorCurrentAmps = armSim.getCurrentDrawAmps();
		data.backMotorCurrentAmps = data.frontMotorCurrentAmps;

		// Sim has no temp
		data.frontMotorTempCelcius = 0;
		data.backMotorTempCelcius = 0;
	}

	/**
	 * Run the motor at the specified voltage.
	 *
	 * @param volts
	 */
	@Override
	public void setVoltage(double volts) {
		inputVolts = MathUtil.applyDeadband(inputVolts, 0.05);
		inputVolts = MathUtil.clamp(volts, -12, 12);
		armSim.setInputVoltage(inputVolts);
	}
}
