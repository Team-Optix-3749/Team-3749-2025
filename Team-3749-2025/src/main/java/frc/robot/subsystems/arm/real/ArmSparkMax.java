package frc.robot.subsystems.arm.real;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import frc.robot.subsystems.arm.ArmIO;
import frc.robot.utils.MiscConstants.SimConstants;

/**
 * IO implementation for an arm subsystem's sparkmax
 * 
 * @author Elise Lin
 */
public class ArmSparkMax implements ArmIO {

    private SparkMax motor;
    private SparkMaxConfig config = new SparkMaxConfig();

    private double inputVolts = 0;
    private double previousVelocity = 0;
    private double velocity = 0;

    public ArmSparkMax(int motorId) {

        motor = new SparkMax(motorId, MotorType.kBrushless);
        config.smartCurrentLimit(30, 50);
        config.encoder.inverted(false);
        config.idleMode(IdleMode.kBrake);
        config.encoder.positionConversionFactor(2 * Math.PI);
        config.encoder.velocityConversionFactor(2 * Math.PI / 60.0);
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    }

    /**
     * Takes in the data from ArmData and uses it to update the data on the
     * position of the motor, velocity, acceleration, applied volts on the motor,
     * and the motor temperature
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
        data.currentAmps = motor.getOutputCurrent();
        data.inputVolts = inputVolts;
        data.appliedVolts = motor.getBusVoltage() * motor.getAppliedOutput();
        data.tempCelcius = motor.getMotorTemperature();

    }

    /**
     * Takes the volts parameter, then uses inputVolts to set the motor voltage
     * Clamp takes the volts and makes sure that the amount of volts isn't above or
     * below the boundary for the voltage
     * Deadband makes it so that the value returns 0.0 if the volts is between -0.05
     * to 0.05
     * 
     * @param volts
     */
    @Override
    public void setVoltage(double volts) {
        inputVolts = MathUtil.clamp(volts, -12.0, 12.0);
        inputVolts = MathUtil.applyDeadband(inputVolts, 0.05);
        motor.setVoltage(inputVolts);

    }
}
