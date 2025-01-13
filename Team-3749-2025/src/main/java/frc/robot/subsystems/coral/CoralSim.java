package frc.robot.subsystems.coral;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.utils.MiscConstants.SimConstants;

public class CoralSim implements CoralIO{

    private FlywheelSim motorOne; //need to check in with build on how many motors we have
    private double intakeVolts;

    public CoralSim()
    {
        motorOne = new FlywheelSim(LinearSystemId.createFlywheelSystem(DCMotor.getNEO(1),0.04,1),
        DCMotor.getNEO(1)); 
    }

    @Override
    public void updateData(CoralData data)
    {
        motorOne.update(SimConstants.loopPeriodSec);
        data.hasCoral = false; //big load of not my problem rn
        data.intakeVoltage = intakeVolts;
        data.goalVoltage = intakeVolts; //read coralio for info
        data.intakeTempCelsius = 0;
        data.intakeVelocityRadPerSec = motorOne.getAngularVelocityRadPerSec();
    }

    @Override
    public void setVoltage(double volts)
    {
        volts = MathUtil.clamp(volts, -12, 12);
        intakeVolts = volts;
        motorOne.setInputVoltage(intakeVolts);
    }


    
}
