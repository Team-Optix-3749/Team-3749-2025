package frc.robot.subsystems.rollers.algae.intake;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.rollers.RollerConstants;
import frc.robot.subsystems.rollers.RollerConstants.RollerStates;
import frc.robot.subsystems.rollers.algae.intake.AlgaeIO.AlgaeIntakeData;
import frc.robot.subsystems.rollers.algae.intake.AlgaeIO.AlgaeWristData;
import frc.robot.subsystems.rollers.coral.CoralIO.CoralData;

public class AlgaeIntake extends SubsystemBase {

    private AlgaeIntake wristData = new AlgaeWristData();
    private AlgaeIO wristIO;  



    //intake data but oh well
    private AlgaeIO intakeIO;

    private double wrist = 0; //consider adding resting voltage

    private RollerStates coralState = RollerStates.STOP; //btw if this isn't obvious the goal velocity and coral state are 
    //the same for both cause why would we ever spin both at different speeds / stop only one side

    public Algae()
    {
        if(Robot.isReal())
        {
            wristIO = new (true);
            rightCoralIO = new CoralReal(false);
            return;
        }
        leftCoralIO = new CoralSim();
        rightCoralIO = new CoralSim();
    }

    public void setGoalVelocity(double velocity)
    {
        goalVelocity = velocity;
    }

    private void setVelocity()
    {
        leftCoralIO.setVoltage(RollerConstants.CoralConstants.coralPIDController.calculate(
        leftData.velocityRadPerSec,goalVelocity) + RollerConstants.CoralConstants.coralFeedForward.calculate(
        goalVelocity));

        //although i would assume it's not an issue, this uses the same PID controller 

        rightCoralIO.setVoltage(RollerConstants.CoralConstants.coralPIDController.calculate(
        rightData.velocityRadPerSec,goalVelocity) + RollerConstants.CoralConstants.coralFeedForward.calculate(
        goalVelocity));
    }

    public void stop()
    {
        goalVelocity = 0;
        leftCoralIO.stop();
        rightCoralIO.stop();
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
        setVelocity(); //if you remove this the wheels maintain inertia
    }

    @Override
    public void periodic()
    {
        runCoralState();
        leftCoralIO.updateData(leftData);
        rightCoralIO.updateData(rightData);
        SmartDashboard.putNumber("CoralVelocity", leftData.velocityRadPerSec);
    }


    
}
