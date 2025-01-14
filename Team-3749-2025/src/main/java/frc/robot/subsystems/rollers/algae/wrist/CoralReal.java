package frc.robot.subsystems.rollers.algae.intake;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

public class CoralReal implements AlgaeWristIO {
    private SparkMax motor = new SparkMax(3000,MotorType.kBrushless); //obviously dont leave that as 3000
    private SparkMaxConfig config = new SparkMaxConfig();
    private RelativeEncoder encoder = motor.getEncoder();
    private double goalVolts = 0;

    public CoralReal(boolean isInverted)
    {
        config.encoder.inverted(isInverted);
        motor.configure(config,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters); //i can't read the
        //board so if this is wrong oh well
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
    public void updateData(CoralData data)
    {
        data.tempCelsius = motor.getMotorTemperature();
        data.velocityRadPerSec = (encoder.getVelocity()*Math.PI*2)/60.0; //convert RPM to RPS
        data.goalVoltage = goalVolts;
        data.busVoltage = motor.getBusVoltage(); //read coralIO for more info
        data.hasCoral = false; //depends on if build does smth like this again
    }

    
}
