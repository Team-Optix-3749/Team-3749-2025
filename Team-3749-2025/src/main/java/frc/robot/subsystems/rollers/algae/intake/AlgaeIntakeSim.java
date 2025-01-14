package frc.robot.subsystems.rollers.algae.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.utils.MiscConstants.SimConstants;

public class AlgaeIntakeSim implements AlgaeIntakeIO {

    private FlywheelSim motor;
    private double intakeVolts;

    public AlgaeIntakeSim()
    {
        motor = new FlywheelSim(LinearSystemId.createFlywheelSystem(DCMotor.getNEO(1),0.04,1),
        DCMotor.getNEO(1));  
    }

    @Override
    public void updateData(CoralData data)
    {
        motor.update(SimConstants.loopPeriodSec);
        data.hasCoral = false; //big load of not my problem rn
        data.busVoltage = intakeVolts;
        data.goalVoltage = intakeVolts; //read coralio for info
        data.tempCelsius = 0;
        data.velocityRadPerSec = motor.getAngularVelocityRadPerSec();
    }

    @Override
    public void setVoltage(double volts)
    {
        volts = MathUtil.clamp(volts, -12, 12);
        intakeVolts = volts;
        motor.setInputVoltage(intakeVolts);
    }

    @Override
    public void stop()
    {
        intakeVolts = 0;
        motor.setInputVoltage(0);
    }
}
