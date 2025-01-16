package frc.robot.subsystems.roller;

import frc.robot.subsystems.roller.RollerIO.RollerData;

public class ScoringRoller extends Roller {
    private RollerIO rollerIO;
    private RollerData rollerData;

    public ScoringRoller(RollerIO rollerIO) {
        super(rollerIO);
    }

    @Override
    public void run() {
        if (!rollerData.sensorTripped) {
            rollerIO.setVoltage(RollerConstants.scoringVoltage);
        } else {
            rollerIO.setVoltage(0.0);
        }
    }
}
