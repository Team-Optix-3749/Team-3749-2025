package frc.robot.subsystems.roller.implementations;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.robot.subsystems.roller.Roller;
import frc.robot.subsystems.roller.RollerConstants;
import frc.robot.subsystems.roller.RollerConstants.Implementations;
import frc.robot.subsystems.roller.RollerIO.RollerData;
import frc.robot.utils.LoggedTunableNumber;

public class ScoringRoller extends Roller {
    private RollerData rollerData;
    
    public ScoringRoller() {
        super(Implementations.SCORING, FF());
        this.rollerData = new RollerData();
        kp = new LoggedTunableNumber(getName() + "/kP", RollerConstants.Scoring.kPVelocity);
        ki = new LoggedTunableNumber(getName() + "/kI", RollerConstants.Scoring.kIVelocity);
        kd = new LoggedTunableNumber(getName() + "/kD", RollerConstants.Scoring.kDVelocity);
        kv = new LoggedTunableNumber(getName() + "/kV", RollerConstants.Scoring.kVVelocity);
        ka = new LoggedTunableNumber(getName() + "/kA", RollerConstants.Scoring.kAVelocity);
        ks = new LoggedTunableNumber(getName() + "/kS", RollerConstants.Scoring.kSVelocity);
        maxVelocity = new LoggedTunableNumber(getName() + "/maxVelocity", RollerConstants.Scoring.maxVelocity);
        maxAcceleration = new LoggedTunableNumber(getName() + "/maxAcceleration", RollerConstants.Scoring.maxAcceleration);
    }

    public static SimpleMotorFeedforward FF() {
        return new SimpleMotorFeedforward(RollerConstants.Scoring.kSVelocity, RollerConstants.Scoring.kVVelocity, RollerConstants.Scoring.kAVelocity);
    }


    @Override
    public void run() {
        if (!rollerData.sensorTripped) {
            setVelocity(RollerConstants.Scoring.velocity);
        } else {
            setVoltage(0.0);
        }
    }
}
