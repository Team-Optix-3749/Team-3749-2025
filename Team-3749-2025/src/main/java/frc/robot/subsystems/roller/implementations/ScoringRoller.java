package frc.robot.subsystems.roller.implementations;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.robot.Robot;
import org.littletonrobotics.junction.Logger;

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
import frc.robot.subsystems.roller.real.JTVisiSight;
import frc.robot.subsystems.roller.sim.PhotoelectricSim;
import frc.robot.subsystems.roller.sim.RollerSim;

/**
 * Scoring implementation of the roller subsystem
 *
 * @author Lilian Wu
 */
public class ScoringRoller extends Roller {
    private RollerData rollerData;
    private PhotoelectricData photoelectricData = new PhotoelectricData();
    private PhotoelectricIO photoelectricIO;
    private boolean hasPiece = true;
    private boolean routineStarted = false;

    public ScoringRoller() {
        super(Implementations.SCORING, FF(), positionPID(), velocityPID());
        this.rollerData = new RollerData();
        if (Robot.isSimulation()) {
            this.photoelectricIO = new PhotoelectricSim();
            photoelectricIO.setInitialState(true);
        } else {
            this.photoelectricIO = new JTVisiSight();
        }
    }

    public static SimpleMotorFeedforward FF() {
        return new SimpleMotorFeedforward(RollerConstants.Scoring.kSVelocity.get(),
                RollerConstants.Scoring.kVVelocity.get(),
                RollerConstants.Scoring.kAVelocity.get());
    }

    public static PIDController positionPID() {
        return new PIDController(RollerConstants.Scoring.kPPosition.get(), RollerConstants.Scoring.kIPosition.get(),
                RollerConstants.Scoring.kDPosition.get());
    }

    public static PIDController velocityPID() {
        return new PIDController(RollerConstants.Scoring.kPVelocity.get(), RollerConstants.Scoring.kIVelocity.get(),
                RollerConstants.Scoring.kDVelocity.get());
    }


    @Override
    public void outtake() {
        setVelocity(RollerStates.OUTTAKE.scoringVelocity);
    }


    public static PIDController positionController() {
        return new PIDController(RollerConstants.Scoring.kPPosition.get(), RollerConstants.Scoring.kIPosition.get(),
                RollerConstants.Scoring.kDPosition.get());
    }

    /**
     * Implemetation of run method
     */
    @Override
    public void intake() {
        if (!rollerData.sensorTripped) {
            setVelocity(RollerStates.INTAKE.scoringVelocity);
        } else {
            setVoltage(0.0);
        }
    }



    public boolean hasPiece() {
        return hasPiece;
    }

    public void setHasPiece(boolean hasPiece) {
        this.hasPiece = hasPiece;
    }

    @Override
    public void periodic() {
        super.periodic();
      
        photoelectricIO.updateData(photoelectricData);

        hasPiece = photoelectricData.sensing;
        
        Logger.recordOutput("subsystems/roller/ScoringRoller/hasPiece", hasPiece);
        Logger.recordOutput("subsystems/roller/ScoringRoller/setInitalState", routineStarted);

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
            SmartDashboard.putString("scoring roller command", this.getCurrentCommand().getName());
        } else {
            SmartDashboard.putString("scoring roller command", "null");
        }

    }
}
