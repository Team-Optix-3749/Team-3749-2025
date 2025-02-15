package frc.robot.subsystems.arm.coral.sim;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.subsystems.arm.coral.CoralArmIO;
import frc.robot.subsystems.arm.coral.CoralArmConstants;
import frc.robot.utils.MiscConstants.SimConstants;

/**
 * IO implementation for an arm subsystem's simulation
 *
 * @author Weston Gardner
 */
public class CoralArmSim implements CoralArmIO {

	private SingleJointedArmSim armSim;

	private double inputVolts = 0;
	private double previousVelocity = 0;

	/**
	 * creates a new io implementation of a single jointed arm in simulation
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
	public CoralArmSim() {

		System.out.println("[Init] Creating ArmSim");

		armSim = new SingleJointedArmSim(
				DCMotor.getNEO(CoralArmConstants.numMotors),
				CoralArmConstants.armGearing,
				CoralArmConstants.momentOfInertia,
				CoralArmConstants.armLength_meters,
				CoralArmConstants.armMinAngle_degrees * Math.PI / 180,
				CoralArmConstants.armMaxAngle_degrees * Math.PI / 180,
				CoralArmConstants.simulateGravity,
				CoralArmConstants.armStartingAngle_degrees * Math.PI / 180);
	}

	/**
	 * Updates the set of loggable inputs for the sim.
	 *
	 * @param data
	 */
	@Override
	public void updateData(ArmData data) {
		armSim.update(0.02);
		double velocity = armSim.getVelocityRadPerSec();
		data.positionRad = armSim.getAngleRads();
		data.velocityUnits = velocity;
		data.accelerationUnits = (velocity - previousVelocity) / SimConstants.loopPeriodSec;

		data.motorAppliedVolts = inputVolts;
		data.motorCurrentAmps = armSim.getCurrentDrawAmps();

		// Sim has no temp
		data.motorTempCelcius = 0;
		previousVelocity = velocity;
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
