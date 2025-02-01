package frc.robot.subsystems.roller.implementations;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.robot.subsystems.roller.Roller;
import frc.robot.subsystems.roller.RollerConstants;
import frc.robot.subsystems.roller.RollerConstants.Implementations;
import frc.robot.subsystems.roller.RollerIO.RollerData;

public class ScoringRoller extends Roller {
    private RollerData rollerData;
    
    public ScoringRoller() {
        super(Implementations.SCORING, velocityController(), FF(), positionController());
        this.rollerData = new RollerData();
    }

    public static PIDController velocityController() {
        return new PIDController(RollerConstants.Scoring.kPVelocity, RollerConstants.Scoring.kIVelocity, RollerConstants.Scoring.kDVelocity);
    }

    public static SimpleMotorFeedforward FF() {
        return new SimpleMotorFeedforward(RollerConstants.Scoring.kSVelocity, RollerConstants.Scoring.kVVelocity, RollerConstants.Scoring.kAVelocity);
    }

    public static PIDController positionController() {
        return new PIDController(RollerConstants.Scoring.kPPosition, RollerConstants.Scoring.kIPosition, RollerConstants.Scoring.kDPosition);
    }

    @Override
    public void run() {
        if (!rollerData.sensorTripped) {
            setVelocity(RollerConstants.Scoring.velocity);
        } else {
            setVoltage(0.0);
        }
    }

    @Override
    public void score() {
        setVelocity(RollerConstants.Scoring.scoreVelocity);
    }
}
