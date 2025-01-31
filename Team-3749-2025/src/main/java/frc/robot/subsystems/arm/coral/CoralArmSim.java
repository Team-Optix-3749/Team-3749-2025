package frc.robot.subsystems.arm.coral;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.utils.MiscConstants.SimConstants;

/**
 * IO implementation for an arm subsystem's simulation
 *
 * @author Weston Gardner
 */
public class CoralArmSim implements CoralArmIO {

	private SingleJointedArmSim armSim;

	/**
	 * creates a new io implementation of a single jointed arm in simulation
	 * @param numMotors
	 * @param gearing
	 * @param momentOfInertia
	 * @param length_meters
	 * @param minAngle_degrees
	 * @param maxAngle_degrees
	 * @param simulateGravity
	 * @param startingAngle_Degrees
	 */
	public CoralArmSim(
			int numMotors,
			double gearing,
			double momentOfInertia,
			double length_meters,
			double minAngle_degrees,
			double maxAngle_degrees,
			boolean simulateGravity,
			double startingAngle_Degrees) {

		System.out.println("[Init] Creating ArmSim");

		armSim = new SingleJointedArmSim(
				DCMotor.getNEO(numMotors),
				gearing,
				momentOfInertia,
				length_meters,
				minAngle_degrees * Math.PI / 180,
				maxAngle_degrees * Math.PI / 180,
				simulateGravity,
				startingAngle_Degrees * Math.PI / 180);
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
		data.motorAppliedVolts = inputVolts;
		data.motorCurrentAmps = armSim.getCurrentDrawAmps();

		// Sim has no temp
		data.motorTempCelcius = 0;
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
