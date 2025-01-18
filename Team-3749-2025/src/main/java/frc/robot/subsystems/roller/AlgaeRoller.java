package frc.robot.subsystems.roller;
public class AlgaeRoller extends Roller {
    
    public AlgaeRoller(RollerIO rollerIO) {
        super(rollerIO);
    }

    @Override
    public void run() {
        setVelocity(RollerConstants.algaeVelocity);
    }
    
    @Override
    public void maintain() {
        setVelocity(RollerConstants.holdVoltage);
    }

}
