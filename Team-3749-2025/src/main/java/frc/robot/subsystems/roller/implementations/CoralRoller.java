package frc.robot.subsystems.roller.implementations;

import java.lang.Thread.State;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.commands.auto.Autos;
import frc.robot.subsystems.roller.PhotoelectricIO;
import frc.robot.subsystems.roller.Roller;
import frc.robot.subsystems.roller.RollerConstants;
import frc.robot.subsystems.roller.PhotoelectricIO.PhotoelectricData;
import frc.robot.subsystems.roller.RollerConstants.Implementations;
import frc.robot.subsystems.roller.RollerConstants.RollerStates;
import frc.robot.subsystems.roller.RollerIO.RollerData;
import frc.robot.subsystems.roller.sim.PhotoelectricSim;
import frc.robot.utils.ShuffleData;

/**
 * Coral implementation of the roller subsystem
 *
 * @author Lilian Wu
 */
public class CoralRoller extends Roller {
    private double lastVelocity = 0.0;
    private boolean hasPiece = true;
    private PhotoelectricIO photoelectricIO;
    private PhotoelectricData photoelectricData = new PhotoelectricData();
    private RollerData rollerData = new RollerData();
    private boolean routineStarted = false;

    private ShuffleData<Boolean> hasPieceLog = new ShuffleData<Boolean>(this.getName(), "hasPiece", hasPiece);
    private ShuffleData<Boolean> setInitialStateLog = new ShuffleData<Boolean>(this.getName(), "setInitialState",
            routineStarted);

    public CoralRoller() {
        super(Implementations.CORAL, FF());
        photoelectricIO = new PhotoelectricSim();
    }

    public static PIDController velocityController() {
        return new PIDController(RollerConstants.Coral.kPVelocity, RollerConstants.Coral.kIVelocity,
                RollerConstants.Coral.kDVelocity);
    }

    public static SimpleMotorFeedforward FF() {
        return new SimpleMotorFeedforward(RollerConstants.Coral.kSVelocity, RollerConstants.Coral.kVVelocity,
                RollerConstants.Coral.kAVelocity);
    }

    public static PIDController positionController() {
        return new PIDController(RollerConstants.Coral.kPPosition, RollerConstants.Coral.kIPosition,
                RollerConstants.Coral.kDPosition);
    }

    public boolean getIsStableState() {
        double currentVelocity = rollerData.rollerVelocityRadPerSec;
        double velocityChange = Math.abs(currentVelocity - lastVelocity);
        double velocityChangeThreshold = 0.05; 

        switch(Robot.coralRoller.getState()) {
            case RUN:
                return velocityChange < velocityChangeThreshold;
            case SCORE:
                return velocityChange < velocityChangeThreshold;
            case MAINTAIN:
                return false;
            case STOP:
                return true;
            default:
                return false;
        }
    }

    public boolean hasPiece() {
        if (Robot.isSimulation()) {
            return hasPiece;
        } else {
            if (!getIsStableState()) { 
                hasPiece = true; 
            }
            return hasPiece;
        }
    }

    public boolean getHasPiece() {
        return hasPiece;
    }

    /**
     * Implemetation of run method
     */
    @Override
    public void run() {
        setVelocity(RollerConstants.Coral.velocity);
    }

    /**
     * Implemetation of score method
     */
    @Override
    public void score() {
        setVelocity(RollerConstants.Coral.scoreVelocity);
    }

    @Override
    public void periodic() {
        super.periodic();
        photoelectricIO.updateData(photoelectricData);
        hasPiece = photoelectricData.sensing;
        hasPieceLog.set(hasPiece);
        setInitialStateLog.set(routineStarted);

        // routineStarted is true when the routine begins in Autos
        if (Autos.isRoutineStarted() && !routineStarted) {
            routineStarted = true;
            // sets initial state at the start of each routine
            photoelectricIO.setInitialState(true);
        }

        // routineStarted is false when the routine ends in Autos
        if (!Autos.isRoutineStarted() && routineStarted) {
            routineStarted = false;
        }

        if (this.getCurrentCommand() != null) {
            SmartDashboard.putString("coral roller command", this.getCurrentCommand().getName());
        } else {
            SmartDashboard.putString("coral roller command", "null");
        }
    }
}
