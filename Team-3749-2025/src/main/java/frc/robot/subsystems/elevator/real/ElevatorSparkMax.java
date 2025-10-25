package frc.robot.subsystems.elevator.real;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.utils.OptixSpark;
import frc.robot.utils.UtilityFunctions;
import frc.robot.utils.MiscConstants.MotorControllerConstants;
import frc.robot.utils.MiscConstants.SimConstants;

/**
 * SparkMax for elevator subsystem
 * 
 * @author Dhyan Soni
 */

public class ElevatorSparkMax implements ElevatorIO {
    private OptixSpark backMotor = new OptixSpark(ElevatorConstants.ElevatorSpecs.motorIds[0],
            OptixSpark.Type.SPARKMAX);

    private OptixSpark frontMotor = new OptixSpark(ElevatorConstants.ElevatorSpecs.motorIds[1],
            OptixSpark.Type.SPARKMAX);

    private double previousVelocity = 0;
    @SuppressWarnings("unused")
    private double inputVolts = 0;

    public ElevatorSparkMax() {
        // System.out.println("[Init] Creating Elevator");
        // 2x for two stages, 2pi for circumfrence, radius of sprocket, gear ratio
        backMotor.setPositionConversionFactor(2 * Math.PI * 2 * ElevatorConstants.ElevatorSpecs.drumRadiusMeters
                / ElevatorConstants.ElevatorSpecs.gearing);
        backMotor.setVelocityConversionFactor(2 * Math.PI * 2 * ElevatorConstants.ElevatorSpecs.drumRadiusMeters
                / (60 * ElevatorConstants.ElevatorSpecs.gearing));

        backMotor.setCurrentLimit(MotorControllerConstants.standardStallLimit,
                MotorControllerConstants.standardFreeLimit);
        backMotor.setInverted(ElevatorConstants.ElevatorSpecs.motorInverted[0]);
        backMotor.setBrakeMode(false);

        backMotor.applyConfig();
        frontMotor.applyConfig(backMotor.getConfig());
        frontMotor.setInverted(ElevatorConstants.ElevatorSpecs.motorInverted[1]);
        frontMotor.applyConfig();

    
    }

    @Override
    public void setBrakeMode(boolean enabled) {
        frontMotor.setBrakeMode(enabled);
        backMotor.setBrakeMode(enabled);
    }

    @Override
    public void updateData(ElevatorData data) {
        double velocity = (backMotor.getVelocity() + frontMotor.getVelocity()) / 2;
        data.positionMeters = (backMotor.getPosition() + frontMotor.getPosition()) / 2;
        Logger.recordOutput("Elevator/backPos", backMotor.getPosition());
        Logger.recordOutput("Elevator/frontPos", frontMotor.getPosition());

        data.velocityMetersPerSecond = velocity;
        data.accelerationMetersPerSecondSquared = (velocity - previousVelocity) / SimConstants.loopPeriodSec;
        data.leftCurrentAmps = backMotor.getCurrent();
        data.rightCurrentAmps = frontMotor.getCurrent();
        data.leftAppliedVolts = backMotor.getAppliedVolts();
        data.rightAppliedVolts = frontMotor.getAppliedVolts();
        data.leftTempCelcius = backMotor.getTemperature();
        data.rightTempCelcius = frontMotor.getTemperature();

        previousVelocity = velocity;
    }

    @Override
    public void setVoltage(double volts) {
        inputVolts = MathUtil.clamp(volts, -12, 12);
        inputVolts = UtilityFunctions.applyDeadband(inputVolts, MotorControllerConstants.deadbandVoltage);
        Logger.recordOutput("Elevator/input volts", inputVolts);

        backMotor.setVoltage(inputVolts);
        frontMotor.setVoltage(inputVolts);
    }
}
