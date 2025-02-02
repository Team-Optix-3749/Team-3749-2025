package frc.robot.subsystems.roller.implementations;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.roller.PhotoelectricIO;
import frc.robot.subsystems.roller.Roller;
import frc.robot.subsystems.roller.RollerConstants;
import frc.robot.subsystems.roller.PhotoelectricIO.PhotoelectricData;
import frc.robot.subsystems.roller.RollerConstants.Implementations;
import frc.robot.subsystems.roller.RollerIO.RollerData;
import frc.robot.subsystems.roller.sim.PhotoelectricSim;

public class CoralRoller extends Roller {
    private double lastVelocity = 0.0;
    private boolean hasPiece = false;
    private PhotoelectricIO photoelectricIO;
    private PhotoelectricData photoelectricData = new PhotoelectricData();
    private RollerData data = new RollerData();
    
    public CoralRoller() {
        super(Implementations.CORAL, velocityController(), FF(), positionController());
        this.photoelectricIO = new PhotoelectricSim();
        photoelectricIO.setInitialState(true);
    }

    public static PIDController velocityController() {
        return new PIDController(RollerConstants.Coral.kPVelocity, RollerConstants.Coral.kIVelocity, RollerConstants.Coral.kDVelocity);
    }

    public static SimpleMotorFeedforward FF() {
        return new SimpleMotorFeedforward(RollerConstants.Coral.kSVelocity, RollerConstants.Coral.kVVelocity, RollerConstants.Coral.kAVelocity);
    }

    public static PIDController positionController() {
        return new PIDController(RollerConstants.Coral.kPPosition, RollerConstants.Coral.kIPosition, RollerConstants.Coral.kDPosition);
    }

    public boolean getIsStableState() {
        double currentVelocity = data.rollerVelocityRadPerSec;
        double velocityChange = Math.abs(currentVelocity - lastVelocity);
        double velocityChangeThreshold = 0.5; // placeholder
        return velocityChange < velocityChangeThreshold;
    }

    public boolean hasPiece() {
        if (Robot.isSimulation()) {
           return hasPiece; 
        } else {
            if (!getIsStableState() && lastVelocity > 0.1) { 
                hasPiece = true; 
            }
            return hasPiece;
        }
    }

    public boolean getHasPiece() {
        return hasPiece;
    }

    @Override
    public void intake() {
        setVelocity(RollerConstants.Coral.velocity);
    }

    @Override
    public void score() {
        setVelocity(RollerConstants.Coral.scoreVelocity);
    }

    public Command getCurrentCommand(){
        return this.getCurrentCommand();
    }

    @Override
    public void periodic() {
        super.periodic();
        photoelectricIO.updateData(photoelectricData);

        if (photoelectricData.sensing) {
            hasPiece = true;
        } else {
            hasPiece = false;
        }
    }
}
