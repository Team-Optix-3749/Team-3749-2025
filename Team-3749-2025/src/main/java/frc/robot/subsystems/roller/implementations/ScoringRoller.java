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
import frc.robot.subsystems.roller.real.JTVisiSight;
import frc.robot.subsystems.roller.sim.PhotoelectricSim;

public class ScoringRoller extends Roller {
    private RollerData rollerData;
    private PhotoelectricData photoelectricData = new PhotoelectricData();
    private PhotoelectricIO photoelectricIO;
    private boolean hasPiece = true;

    public ScoringRoller() {
        super(Implementations.SCORING, velocityController(), FF(), positionController());
        this.rollerData = new RollerData();
        if (Robot.isSimulation()) {
            this.photoelectricIO = new PhotoelectricSim();
            photoelectricIO.setInitialState(true);
        } else {
            this.photoelectricIO = new JTVisiSight(); 
        }
    }

    public static PIDController velocityController() {
        return new PIDController(RollerConstants.Scoring.kPVelocity, RollerConstants.Scoring.kIVelocity,
                RollerConstants.Scoring.kDVelocity);
    }

    public static SimpleMotorFeedforward FF() {
        return new SimpleMotorFeedforward(RollerConstants.Scoring.kSVelocity, RollerConstants.Scoring.kVVelocity,
                RollerConstants.Scoring.kAVelocity);
    }

    public static PIDController positionController() {
        return new PIDController(RollerConstants.Scoring.kPPosition, RollerConstants.Scoring.kIPosition,
                RollerConstants.Scoring.kDPosition);
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

    public boolean hasPiece() {
        return photoelectricData.sensing;
    }

    public void setHasPiece(boolean hasPiece) {
        this.hasPiece = hasPiece;
    }

    // public Command getCurrentCommand(){
    //     if (this.getCurrentCommand() != null) {
    //         return this.getCurrentCommand();
    //     }
    //     return null;
    // }

    @Override
    public void periodic() {
        super.periodic();
        photoelectricIO.updateData(photoelectricData);

        System.out.println("sensing" + photoelectricData.sensing);
        if (photoelectricData.sensing) {
            hasPiece = true;
        } else {
        
            hasPiece = false;
        }
    }
}
