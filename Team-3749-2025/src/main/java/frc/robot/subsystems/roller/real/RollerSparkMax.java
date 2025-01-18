package frc.robot.subsystems.roller.real;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import frc.robot.subsystems.roller.RollerIO;

public class RollerSparkMax implements RollerIO {
    private final SparkMax rollerMotor;
    private final RelativeEncoder relativeEncoder;
    private double rollerGoalVolts = 0.0;

    public RollerSparkMax(int motorID) {
        rollerMotor = new SparkMax(motorID, MotorType.kBrushless);
        relativeEncoder = rollerMotor.getEncoder();
    }
    
    @Override
    public void updateData(RollerData data) {
        data.rollerAppliedVolts = rollerMotor.getBusVoltage() * rollerMotor.getAppliedOutput();
        data.rollerVelocityRadPerSec = relativeEncoder.getVelocity();
        data.rollerTempCelcius = rollerMotor.getMotorTemperature();
    }

    @Override
    public void setVoltage(double volts) {
        rollerGoalVolts = MathUtil.clamp(volts, -12, 12);
        rollerMotor.setVoltage(rollerGoalVolts);
    }
}
