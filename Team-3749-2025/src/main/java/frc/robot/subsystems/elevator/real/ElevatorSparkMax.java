package frc.robot.subsystems.elevator.real;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.utils.MiscConstants.SimConstants;

/**
 * SparkMax for elevator subsystem
 * 
 * @author Dhyan Soni
 */

public class ElevatorSparkMax implements ElevatorIO {
    //motors
    private SparkMax leftMotor = new SparkMax(ElevatorConstants.ElevatorSpecs.motorIds[0], MotorType.kBrushless);
    private SparkMax rightMotor = new SparkMax(ElevatorConstants.ElevatorSpecs.motorIds[1], MotorType.kBrushless);
    private SparkMaxConfig leftConfig = new SparkMaxConfig();
    private SparkMaxConfig rightConfig = new SparkMaxConfig();

    private SparkAbsoluteEncoder absoluteEncoder;
    private double absolutePos;

    private double inputVolts = 0;
    private double previousVelocity = 0;
    private double velocity = 0;

    public ElevatorSparkMax() {
        System.out.println("[Init] Creating Elevator");

        //creates left motor
        leftConfig.smartCurrentLimit(ElevatorConstants.ElevatorSpecs.stallLimit,
                ElevatorConstants.ElevatorSpecs.freeLimit);
        leftConfig.encoder.inverted(false);
        leftConfig.inverted(false);
        leftConfig.idleMode(IdleMode.kBrake);
        leftConfig.encoder.positionConversionFactor(2 * Math.PI);
        leftConfig.encoder.velocityConversionFactor(2 * Math.PI / 60.0);
        leftMotor.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        //sets right motor to inverted left motor 
        rightConfig.apply(leftConfig);
        rightConfig.encoder.inverted(true);
        rightConfig.inverted(true);
        rightMotor.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        //gets the position
        absoluteEncoder = leftMotor.getAbsoluteEncoder();
        absolutePos = absoluteEncoder.getPosition() + ElevatorConstants.ElevatorSpecs.zeroOffset;

        leftMotor.getEncoder().setPosition(absolutePos);
        rightMotor.getEncoder().setPosition(absolutePos);
    }


    public void setIdleMode(IdleMode idleMode) {
        leftConfig.idleMode(idleMode);
        rightConfig.idleMode(idleMode);
        leftMotor.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rightMotor.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    //updates the loggable data
    @Override
    public void updateData(ElevatorData data) {
        //updates velocity, position, and acceleration data
        previousVelocity = velocity;
        velocity = (leftMotor.getEncoder().getVelocity() + rightMotor.getEncoder().getVelocity()) / 2;
        // data.positionMeters = (leftMotor.getEncoder().getPosition() + rightMotor.getEncoder().getVelocity()) / 2
        //         + absolutePos;
        data.positionMeters = (leftMotor.getEncoder().getPosition() + rightMotor.getEncoder().getVelocity()) / 2;
        data.velocityMetersPerSecond = velocity;
        data.accelerationMetersPerSecondSquared = (velocity - previousVelocity) / SimConstants.loopPeriodSec;

        //updates data for right and left motor
        data.leftCurrentAmps = leftMotor.getOutputCurrent();
        data.rightCurrentAmps = rightMotor.getOutputCurrent();
        data.inputVolts = inputVolts;
        data.leftAppliedVolts = leftMotor.getBusVoltage() * leftMotor.getAppliedOutput();
        data.rightAppliedVolts = rightMotor.getBusVoltage() * rightMotor.getAppliedOutput();
        data.leftTempCelcius = leftMotor.getMotorTemperature();
        data.rightTempCelcius = rightMotor.getMotorTemperature();
    }

    /**Sets the voltage for the left and right motors */
    @Override
    public void setVoltage(double volts) {
        inputVolts = MathUtil.applyDeadband(inputVolts, 0.05);
        inputVolts = MathUtil.clamp(volts, -12, 12);
        leftMotor.setVoltage(inputVolts);
        rightMotor.setVoltage(inputVolts);
    }
}
 