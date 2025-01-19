package frc.robot.subsystems.roller.implementations;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.robot.subsystems.roller.Roller;
import frc.robot.subsystems.roller.RollerConstants;
import frc.robot.subsystems.roller.RollerConstants.Implementations;

public class AlgaeRoller extends Roller {

    public AlgaeRoller() {
        super(Implementations.ALGAE, velocityController(), FF(), positionController()); 
    }

    public static PIDController velocityController() {
        return new PIDController(RollerConstants.Algae.kPVelocity, RollerConstants.Algae.kIVelocity, RollerConstants.Algae.kDVelocity);
    }

    public static SimpleMotorFeedforward FF() {
        return new SimpleMotorFeedforward(RollerConstants.Algae.kSVelocity, RollerConstants.Algae.kVVelocity, RollerConstants.Algae.kAVelocity);
    }

    public static PIDController positionController() {
        return new PIDController(RollerConstants.Algae.kPPosition, RollerConstants.Algae.kIPosition, RollerConstants.Algae.kDPosition);
    }

    @Override
    public void run() {
        setVelocity(RollerConstants.Algae.velocity);
    }
    
}
