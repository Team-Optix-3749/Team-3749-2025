package frc.robot.subsystems.roller;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.roller.RollerIO.RollerData;
import frc.robot.subsystems.roller.RollerConstants.RollerStates;

public abstract class Roller extends SubsystemBase {
    private RollerIO rollerIO;
    private RollerData rollerData;
    private RollerStates rollerStates;
    private final PIDController positionController;
    private double lastKnownVelocity = 0.0;

    public Roller(RollerIO rollerIO) {
        this.rollerIO = rollerIO;
        this.positionController = new PIDController(0.1, 0, 0);
        rollerData = new RollerData();
    }
    
    public void setVoltage(double volts) {
        rollerIO.setVoltage(volts);
    }

    public void setState(RollerStates rollerStates) {
        this.rollerStates = rollerStates;
    }

    public void runRollerStates() {
        switch(rollerStates) {
            case RUN:
                run();
            case MAINTAIN:
                lastKnownVelocity = rollerData.rollerVelocityRadPerSec;
                maintain();
            case STOP:
                stop();
        }
    }

    public abstract void run();

    public void maintain() {
        double holdVoltage = positionController.calculate(rollerData.rollerVelocityRadPerSec, lastKnownVelocity);
        rollerIO.setVoltage(holdVoltage);
    }

    public void stop() {
        rollerIO.setVoltage(0.0);
    }

    // public abstract void handleSpecialBehavior();

    @Override
    public void periodic() {
        rollerIO.updateData(rollerData);
    }

}
