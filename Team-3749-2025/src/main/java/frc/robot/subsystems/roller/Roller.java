package frc.robot.subsystems.roller;

import java.util.Map;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.roller.RollerIO.RollerData;
import frc.robot.subsystems.roller.real.RollerSparkMax;
import frc.robot.subsystems.roller.sim.RollerSim;
import frc.robot.utils.LoggedTunableNumber;
import frc.robot.utils.MotorData;
import frc.robot.utils.SysIdTuner;
import frc.robot.utils.SysIdTuner.Type;
import frc.robot.Robot;
import frc.robot.subsystems.roller.RollerConstants.Implementations;
import frc.robot.subsystems.roller.RollerConstants.RollerStates;

import static edu.wpi.first.units.Units.*;

public abstract class Roller extends SubsystemBase {
    private RollerIO rollerIO;
    private RollerData rollerData = new RollerData();
    private RollerStates rollerState;
    private SimpleMotorFeedforward rollerFF;
    private PIDController positionController;
    private PIDController velocityController;

    private double lastKnownPosition = 0.0;

    protected LoggedTunableNumber kv;
    protected LoggedTunableNumber ka;
    protected LoggedTunableNumber ks;
    protected LoggedTunableNumber maxVelocity;
    protected LoggedTunableNumber maxAcceleration;
    // SysID
    Map<String, MotorData> motorData = Map.of(
            "roller_motor", new MotorData(
                    rollerData.rollerAppliedVolts,
                    rollerData.rollerPositionRad,
                    rollerData.rollerVelocityRadPerSec,
                    0));

    private SysIdRoutine.Config config = new SysIdRoutine.Config(
            Volts.per(Seconds).of(1), // Voltage ramp rate
            Volts.of(12), // Max voltage
            Seconds.of(12) // Test duration
    );

    private SysIdTuner sysIdTuner;

    public Roller(Implementations implementation, SimpleMotorFeedforward rollerFF, PIDController positionController,
            PIDController velocityController) {
        rollerIO = Robot.isSimulation() ? new RollerSim(implementation)
                : new RollerSparkMax(implementation);

        String name = implementation.name();
        this.rollerFF = rollerFF;
        this.positionController = positionController;
        this.velocityController = velocityController;
        this.rollerState = RollerConstants.RollerStates.STOP;

        sysIdTuner = new SysIdTuner("roller " + name, config, this, rollerIO::setVoltage, motorData, Type.ROTATIONAL);
    }

    public SysIdTuner getSysIdTuner() {
        return sysIdTuner;
    }

    public RollerIO getRollerIO() {
        return rollerIO;
    }

    public void setVoltage(double volts) {
        System.out.println(volts);
        rollerIO.setVoltage(volts);
    }

    public void setVelocity(double velocityRadPerSec) {
        double PIDOutput = velocityController.calculate(rollerData.rollerVelocityRadPerSec, velocityRadPerSec);
        double FFOutput = rollerFF.calculate(velocityRadPerSec);
        rollerIO.setVoltage(PIDOutput + FFOutput);
        // rollerIO.setVelocity(velocityRadPerSec,
        // rollerFF.calculate(velocityRadPerSec));
    }

    public RollerStates getState() {
        return rollerState;
    }

    public void setState(RollerStates rollerState) {
        System.out.println(rollerState.name());
        this.rollerState = rollerState;
        if (rollerState == RollerConstants.RollerStates.MAINTAIN) {
            lastKnownPosition = rollerData.rollerPositionRad;
        }
    }

    public void runRollerStates() {
        switch (rollerState) {
            case RUN:
                run();
                break;
            case MAINTAIN:
                maintain();
                break;
            case STOP:
                stop();
                break;
            case OUTTAKE:
                outtake();
                break;
        }
    }
    public abstract void outtake();
    public abstract void run();

    public void maintain() {

        rollerIO.setPosition(rollerData.rollerPositionRad, lastKnownPosition);

    }

    public void stop() {
        rollerIO.setVoltage(0.0);
    }

    @Override
    public void periodic() {
        rollerIO.updateData(rollerData);
        runRollerStates();

        motorData.get("roller_motor").velocity = rollerData.rollerVelocityRadPerSec;
        motorData.get("roller_motor").position = rollerData.rollerPositionRad;
        motorData.get("roller_motor").acceleration = rollerData.rollerVelocityRadPerSec;
        motorData.get("roller_motor").appliedVolts = rollerData.rollerAppliedVolts;

        Logger.recordOutput("subsystems/roller/" + getName() + "/velocity", rollerData.rollerVelocityRadPerSec);
        Logger.recordOutput("subsystems/roller/" + getName() + "/applied voltage", rollerData.rollerAppliedVolts);
        Logger.recordOutput("subsystems/roller/" + getName() + "/current", rollerData.currentAmps);
        Logger.recordOutput("subsystems/roller/" + getName() + "/position", rollerData.rollerPositionRad);
        Logger.recordOutput("subsystems/roller/" + getName() + "/last known position", lastKnownPosition);
        Logger.recordOutput("subsystems/roller/" + getName() + "/state", rollerState.name());
        Logger.recordOutput("subsystems/roller/" + getName() + "/acceleration", rollerData.acceleration);
    }

}
