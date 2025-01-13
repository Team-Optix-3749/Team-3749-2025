package frc.robot.subsystems.coral;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.coral.CoralIO.CoralData;

public class Coral extends SubsystemBase {

    private CoralData coralData = new CoralData();
    private double goalVelocityRadPerSec = 0; //consider adding resting voltage
    private CoralIO coralIO;

    public Coral()
    {
        if(Robot.isReal())
        {
            coralIO = new CoralReal();
            return;
        }
        coralIO = new CoralSim();
    }

    public void setGoalVelocity(double velocity)
    {
        goalVelocityRadPerSec = velocity;
        coralIO.setVoltage(0);
    }

    private void setVelocity()
    {
        coralIO.setVoltage(CoralConstants.coralPIDController.calculate(coralData.intakeVelocityRadPerSec,goalVelocityRadPerSec));
    }

    public void stop()
    {
        goalVelocityRadPerSec = 0;
    }

    @Override
    public void periodic()
    {
        coralIO.updateData(coralData);
        setVelocity();
        SmartDashboard.putNumber("CoralVelocity", coralData.intakeVelocityRadPerSec);
    }


    
}
