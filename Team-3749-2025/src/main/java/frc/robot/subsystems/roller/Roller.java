package frc.robot.subsystems.roller;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.roller.sim.RollerSim;

public class Roller extends SubsystemBase {
    private RollerIO rollerIO;
    private RollerIO.RollerData rollerData;

    public Roller(RollerIO rollerIO) {
        this.rollerIO = rollerIO;
        rollerData = new RollerIO.RollerData();
    }

    public void setVoltage(double volts) {
        rollerIO.setVoltage(volts);
    }

    public void update() {
        rollerIO.updateData(rollerData);
    }

    // public abstract void handleSpecialBehavior();

    @Override
    public void periodic() {
        update();
    }

}
