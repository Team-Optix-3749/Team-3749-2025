package frc.robot.subsystems.roller;
public class AlgaeRoller extends Roller {
    
    private RollerIO rollerIO;

    public AlgaeRoller(RollerIO rollerIO) {
        super(rollerIO);
    }

    @Override
    public void run() {
        rollerIO.setVoltage(RollerConstants.runVoltage);
    }
    @Override
    public void maintain() {
        rollerIO.setVoltage(RollerConstants.holdVoltage);
    }

}
