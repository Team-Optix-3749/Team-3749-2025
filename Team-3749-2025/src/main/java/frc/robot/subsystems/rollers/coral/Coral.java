package frc.robot.subsystems.rollers.coral;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.rollers.RollerConstants;
import frc.robot.subsystems.rollers.RollerConstants.RollerStates;
import frc.robot.subsystems.rollers.coral.CoralIO.CoralData;

public class Coral extends SubsystemBase {

    private CoralData coralData = new CoralData();
    private double goalVelocityRadPerSec = 0; //consider adding resting voltage
    private CoralIO coralIO;
    private RollerStates coralState = RollerStates.STOP;

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
    }

    private void setVelocity()
    {
        coralIO.setVoltage(RollerConstants.CoralConstants.coralPIDController.calculate(
        coralData.intakeVelocityRadPerSec,goalVelocityRadPerSec) + RollerConstants.CoralConstants.coralFeedForward.calculate(
        goalVelocityRadPerSec));
    }

    public void stop()
    {
        goalVelocityRadPerSec = 0;
        coralIO.setVoltage(0);
    }

    public void setCoralState(RollerStates state)
    {
        coralState = state;
    }

    private void runCoralState()
    {
        switch(coralState)
        {
            case RUN:
                setVelocity();
            break;

            case MAINTAIN:
                //???
            break;

            case STOP:
                stop();
            break;
        }
    }

    @Override
    public void periodic()
    {
        runCoralState();
        coralIO.updateData(coralData);
        SmartDashboard.putNumber("CoralVelocity", coralData.intakeVelocityRadPerSec);
    }


    
}
