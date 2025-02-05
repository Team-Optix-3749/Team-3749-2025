package frc.robot.subsystems.arm.climb.real;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.arm.climb.ClimbArmIO;
import frc.robot.subsystems.arm.climb.ClimbArmIO.ArmData;
import frc.robot.utils.MiscConstants.SimConstants;

/**
 * IO implementation for an arm subsystem's sparkmax
 *
 * @author Elise Lin
 */
public class ClimbArmSparkMax implements ClimbArmIO {

	private SparkMax frontMotor;
	private SparkMax backMotor;
	private SparkMaxConfig frontMotorConfig = new SparkMaxConfig();
	private SparkMaxConfig backMotorConfig = new SparkMaxConfig();

	private SparkAbsoluteEncoder absoluteEncoder;
	private double absolutePos;

	private double inputVolts = 0;
	private double previousVelocity = 0;
	private double velocity = 0;

	/**
	 * creates a new io implementation of a real arm that uses an absolute encoder
	 * with two motors
	 * 
	 * @param frontMotorId
	 * @param backMotorId
	 */
	public ClimbArmSparkMax(int frontMotorId, int backMotorId) {

		frontMotor = new SparkMax(frontMotorId, MotorType.kBrushless);
		backMotor = new SparkMax(backMotorId, MotorType.kBrushless);

		frontMotorConfig.smartCurrentLimit(ArmConstants.NEOStallLimit, ArmConstants.NEOFreeLimit);
		frontMotorConfig.encoder.inverted(false);
		frontMotorConfig.inverted(false);
		frontMotorConfig.idleMode(IdleMode.kBrake);
		frontMotorConfig.encoder.positionConversionFactor(2 * Math.PI);
		frontMotorConfig.encoder.velocityConversionFactor(2 * Math.PI / 60.0);

		frontMotor.configure(frontMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

		backMotorConfig.apply(frontMotorConfig);
		backMotorConfig.encoder.inverted(true);
		backMotorConfig.inverted(true);

		backMotor.configure(backMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

		absoluteEncoder = frontMotor.getAbsoluteEncoder();
		absolutePos = absoluteEncoder.getPosition();

		frontMotor.getEncoder().setPosition(absolutePos);
		backMotor.getEncoder().setPosition(absolutePos);
	}

	public void setIdleMode(IdleMode idleMode) {
		frontMotorConfig.idleMode(idleMode);
		backMotorConfig.idleMode(idleMode);
		frontMotor.configure(frontMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
		backMotor.configure(backMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
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
		velocity = (frontMotor.getEncoder().getVelocity() + backMotor.getEncoder().getVelocity()) / 2;
		data.positionUnits = (frontMotor.getEncoder().getPosition() + backMotor.getEncoder().getVelocity()) / 2;
		data.velocityUnits = velocity;
		data.accelerationUnits = (velocity - previousVelocity) / SimConstants.loopPeriodSec;
		data.frontMotorCurrentAmps = frontMotor.getOutputCurrent();
		data.backMotorCurrentAmps = backMotor.getOutputCurrent();
		data.inputVolts = inputVolts;
		data.frontMotorAppliedVolts = frontMotor.getBusVoltage() * frontMotor.getAppliedOutput();
		data.backMotorAppliedVolts = backMotor.getBusVoltage() * backMotor.getAppliedOutput();
		data.frontMotorTempCelcius = frontMotor.getMotorTemperature();
		data.backMotorTempCelcius = backMotor.getMotorTemperature();
	}

	/**
	 * sets the voltage of the motor
	 * 
	 * @param volts
	 */
	@Override
	public void setVoltage(double volts) {
		inputVolts = MathUtil.applyDeadband(inputVolts, 0.05);
		inputVolts = MathUtil.clamp(volts, -12, 12);
		frontMotor.setVoltage(inputVolts);
		backMotor.setVoltage(inputVolts);
	}
}
