package frc.robot.subsystems.roller;

import frc.robot.subsystems.roller.RollerIO.RollerData;

public class ScoringRoller extends Roller {
    private RollerData rollerData;

    public ScoringRoller(RollerIO rollerIO) {
        super(rollerIO);
        this.rollerData = new RollerData();
    }

    @Override
    public void run() {
        if (!rollerData.sensorTripped) {
            setVelocity(RollerConstants.scoringVelocity);
        } else {
            setVoltage(0.0);
        }
    }
}
