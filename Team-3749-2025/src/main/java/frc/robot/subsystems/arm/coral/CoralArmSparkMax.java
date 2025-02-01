package frc.robot.subsystems.arm.coral;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import frc.robot.subsystems.arm.ClimbArmIO;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.utils.MiscConstants.SimConstants;

/**
 * IO implementation for an arm subsystem's sparkmax
 *
 * @author Elise Lin
 * @author Weston Gardner
 */
public class CoralArmSparkMax implements ClimbArmIO {

	private SparkMax motor;
	private SparkMaxConfig motorConfig = new SparkMaxConfig();

	private SparkAbsoluteEncoder absoluteEncoder;
	private double absolutePos;

	private double inputVolts = 0;
	private double previousVelocity = 0;
	private double velocity = 0;

	public CoralArmSparkMax(int motorId) {

		motor = new SparkMax(motorId, MotorType.kBrushless);

		motorConfig.smartCurrentLimit(ArmConstants.NEOStallLimit, ArmConstants.NEOFreeLimit);
		motorConfig.encoder.inverted(false);
		motorConfig.inverted(false);
		motorConfig.idleMode(IdleMode.kBrake);
		motorConfig.encoder.positionConversionFactor(2 * Math.PI);
		motorConfig.encoder.velocityConversionFactor(2 * Math.PI / 60.0);

		motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

		absoluteEncoder = motor.getAbsoluteEncoder();
		absolutePos = absoluteEncoder.getPosition();

		motor.getEncoder().setPosition(absolutePos);
	}

	public void setIdleMode(IdleMode idleMode) {
		motorConfig.idleMode(idleMode);
		motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
	}

	/**
	 * Takes in the data from ArmData and uses it to update the data on the position
	 * of the motor,
	 * velocity, acceleration, applied volts on the motor, and the motor temperature
	 *
	 * @param data
	 */
	@Override
	public void updateData(ArmData data) {
		previousVelocity = velocity;
		velocity = motor.getEncoder().getVelocity();
		data.positionUnits = motor.getEncoder().getPosition();
		data.velocityUnits = velocity;
		data.accelerationUnits = (velocity - previousVelocity) / SimConstants.loopPeriodSec;
		data.firstMotorCurrentAmps = motor.getOutputCurrent();
		data.inputVolts = inputVolts;
		data.firstMotorAppliedVolts = motor.getBusVoltage() * motor.getAppliedOutput();
		data.firstMotorTempCelcius = motor.getMotorTemperature();
	}

	/**
	 * Takes the volts parameter, then uses inputVolts to set the motor voltage
	 * Clamp takes the volts
	 * and makes sure that the amount of volts isn't above or below the boundary for
	 * the voltage
	 * Deadband makes it so that the value returns 0.0 if the volts is between -0.05
	 * to 0.05
	 *
	 * @param volts
	 */
	@Override
	public void setVoltage(double volts) {
		inputVolts = MathUtil.applyDeadband(inputVolts, 0.05);
		inputVolts = MathUtil.clamp(volts, -12, 12);
		motor.setVoltage(inputVolts);
	}
}
