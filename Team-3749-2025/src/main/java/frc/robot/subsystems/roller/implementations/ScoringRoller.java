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
        super(Implementations.SCORING, velocityController(), FF());
        this.rollerData = new RollerData();
    }

    public static PIDController velocityController() {
        return new PIDController(RollerConstants.Scoring.kPSim, RollerConstants.Scoring.kISim, RollerConstants.Scoring.kDSim);
    }

    public static SimpleMotorFeedforward FF() {
        return new SimpleMotorFeedforward(RollerConstants.Scoring.kPSim, RollerConstants.Scoring.kPSim, RollerConstants.Scoring.kPSim);
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
