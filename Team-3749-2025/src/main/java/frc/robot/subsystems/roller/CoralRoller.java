package frc.robot.subsystems.roller;

public class CoralRoller extends Roller {

    public CoralRoller(RollerIO rollerIO) {
        super(rollerIO);
    }

    @Override
    public void run() {
        setVoltage(RollerConstants.intakeVoltage);
    }
}
