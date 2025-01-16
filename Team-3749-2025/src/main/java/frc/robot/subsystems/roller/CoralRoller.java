package frc.robot.subsystems.roller;

public class CoralRoller extends Roller {
    private RollerIO rollerIO;

    public CoralRoller(RollerIO rollerIO) {
        super(rollerIO);
    }

    @Override
    public void run() {
        rollerIO.setVoltage(RollerConstants.intakeVoltage);
    }
}
