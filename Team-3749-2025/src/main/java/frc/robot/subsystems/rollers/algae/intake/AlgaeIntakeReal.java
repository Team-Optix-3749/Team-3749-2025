package frc.robot.subsystems.rollers.algae.intake;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.MathUtil;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;

public class AlgaeIntakeReal implements AlgaeIntakeIO {
    private SparkMax motor = new SparkMax(3000,MotorType.kBrushless); //obviously dont leave that as 3000
    private RelativeEncoder encoder = motor.getEncoder();
    private double goalVolts = 0;

    public AlgaeIntakeReal()
    {
        
    }

    @Override
    public void stop()
    {
        goalVolts = 0;
        motor.stopMotor();
    }

    @Override
    public void setVoltage(double volts)
    {
        volts = MathUtil.clamp(volts, -12, 12);
        goalVolts = volts;
        motor.setVoltage(volts);
    }

    @Override
    public void updateData(AlgaeIntakeData data)
    {
        data.tempCelsius = motor.getMotorTemperature();
        data.velocityRadPerSec = (encoder.getVelocity()*Math.PI*2)/60.0; //convert RPM to RPS
        data.goalVoltage = goalVolts;
        data.busVoltage = motor.getBusVoltage(); //read coralIO for more info
        data.hasAlgae = false; //depends on if build does smth like this again
    }

    
}
