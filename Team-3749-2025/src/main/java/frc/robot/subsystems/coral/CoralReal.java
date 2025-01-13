package frc.robot.subsystems.coral;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;

public class CoralReal implements CoralIO {
    private SparkMax motorOne = new SparkMax(3000,MotorType.kBrushless); //obviously dont leave that as 3000
    private RelativeEncoder motorOneEncoder = motorOne.getEncoder();
    private double goalVolts = 0;

    public CoralReal()
    {

    }

    @Override
    public void setVoltage(double volts)
    {
        volts = MathUtil.clamp(volts, -12, 12);
        goalVolts = volts;
        motorOne.setVoltage(volts);
    }

    @Override
    public void updateData(CoralData data)
    {
        data.intakeTempCelsius = motorOne.getMotorTemperature();
        data.intakeVelocityRadPerSec = (motorOneEncoder.getVelocity()*Math.PI*2)/60.0; //convert RPM to RPS
        data.goalVoltage = goalVolts;
        data.intakeVoltage = motorOne.getBusVoltage(); //read coralIO for more info
        data.hasCoral = false; //depends on if build does smth like this again
    }

    
}
