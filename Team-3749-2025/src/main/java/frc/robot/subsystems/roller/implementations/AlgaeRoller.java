package frc.robot.subsystems.roller.implementations;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.robot.subsystems.roller.Roller;
import frc.robot.subsystems.roller.RollerConstants;
import frc.robot.subsystems.roller.RollerConstants.Implementations;

public class AlgaeRoller extends Roller {

    public AlgaeRoller() {
        super(Implementations.ALGAE, velocityController(), FF()); 
    }

    public static PIDController velocityController() {
        return new PIDController(RollerConstants.Algae.kPSim, RollerConstants.Algae.kISim, RollerConstants.Algae.kDSim);
    }

    public static SimpleMotorFeedforward FF() {
        return new SimpleMotorFeedforward(RollerConstants.Algae.kPSim, RollerConstants.Algae.kPSim, RollerConstants.Algae.kPSim);
    }

    @Override
    public void run() {
        setVelocity(RollerConstants.Algae.velocity);
    }
    
}
