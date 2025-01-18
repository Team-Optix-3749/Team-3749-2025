package frc.robot.subsystems.roller.implementations;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.robot.subsystems.roller.Roller;
import frc.robot.subsystems.roller.RollerConstants;
import frc.robot.subsystems.roller.RollerConstants.Implementations;

public class CoralRoller extends Roller {
    
    public CoralRoller() {
        super(Implementations.CORAL, feedBack(), FF());
    }

    public static PIDController feedBack() {
        return new PIDController(RollerConstants.Coral.kPSim, RollerConstants.Coral.kISim, RollerConstants.Coral.kDSim);
    }

    public static SimpleMotorFeedforward FF() {
        return new SimpleMotorFeedforward(RollerConstants.Coral.kPSim, RollerConstants.Coral.kPSim, RollerConstants.Coral.kPSim);
    }

    @Override
    public void run() {
        setVelocity(RollerConstants.Coral.velocity);
    }
}
