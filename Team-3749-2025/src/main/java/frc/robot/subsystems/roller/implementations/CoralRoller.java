package frc.robot.subsystems.roller.implementations;

import edu.wpi.first.math.controller.PIDController;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.Timer;
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

/**
 * Coral implementation of the roller subsystem
 *
 * @author Lilian Wu
 */

@SuppressWarnings("unused")
public class CoralRoller extends Roller {
    private double lastVelocity = 0.0;
    private boolean hasPiece = false;
    private PhotoelectricIO photoelectricIO;
    private PhotoelectricData photoelectricData = new PhotoelectricData();
    private RollerData rollerData = new RollerData();
    private boolean routineStarted = false;
    private double intakeTimestamp = 0;
    private double outtakeTimestamp = 0;
    private double scoreTimestamp = 0;

    public CoralRoller() {
        super(Implementations.CORAL, FF(), positionPID(), velocityPID());
        photoelectricIO = new PhotoelectricSim();

        if (Robot.isSimulation()) {
            hasPiece = true;
        }
    }

    public static SimpleMotorFeedforward FF() {
        return new SimpleMotorFeedforward(RollerConstants.Coral.kSVelocity.get(),
                RollerConstants.Coral.kVVelocity.get(),
                RollerConstants.Coral.kAVelocity.get());
    }

    public static PIDController positionPID() {
        return new PIDController(RollerConstants.Coral.kPPosition.get(), RollerConstants.Coral.kIPosition.get(),
                RollerConstants.Coral.kDPosition.get());
    }

    public static PIDController velocityPID() {
        return new PIDController(RollerConstants.Coral.kPVelocity.get(), RollerConstants.Coral.kIVelocity.get(),
                RollerConstants.Coral.kDVelocity.get());
    }

    public boolean hasPiece() {

        if (Robot.isSimulation()) {
            return photoelectricData.sensing;
        } else {
            if (!super.getIsStableState() && getState() == RollerStates.INTAKE
                    && Timer.getFPGATimestamp() - intakeTimestamp > 0.6) {
                hasPiece = true;
            } else if (super.getIsStableState() && getState() == RollerStates.OUTTAKE
                    && Timer.getFPGATimestamp() - outtakeTimestamp > 0.15) {
                hasPiece = false;
            } else if (super.getIsStableState() && getState() == RollerStates.SCORE
                    && Timer.getFPGATimestamp() - outtakeTimestamp > 0.3) {
                hasPiece = false;
            }
            return hasPiece;
        }
    }

    @Override
    public void setState(RollerStates rollerState) {
        System.out.println(rollerState.name());
        super.setState(rollerState);
        if (rollerState.equals(RollerStates.INTAKE)) {
            intakeTimestamp = Timer.getFPGATimestamp();
        } else if (rollerState.equals(RollerStates.OUTTAKE)) {
            outtakeTimestamp = Timer.getFPGATimestamp();
        }

    }

    public void setHasPiece(boolean hasPiece){
        this.hasPiece = hasPiece;
    }

    /**
     * Implemetation of run method
     */
    @Override
    public void intake() {
        setVelocity(RollerStates.INTAKE.coralVelocity);
    }

    /**
     * Implemetation of run method
     */
    @Override
    public void outtake() {
        setVelocity(RollerStates.OUTTAKE.coralVelocity);
    }

    @Override
    public void score() {
        setVelocity(RollerStates.SCORE.coralVelocity);
    }

    @Override
    public void maintain() {

            setVoltage(1.5);
 
        
    }

    @Override
    public void periodic() {
        
        super.periodic();
        photoelectricIO.updateData(photoelectricData);

        Logger.recordOutput("Roller/CoralRoller/hasPiece", hasPiece());
        Logger.recordOutput("Roller/CoralRoller/setInitalState", routineStarted);
        Logger.recordOutput("Roller/CoralRoller/isStableState", super.getIsStableState());

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
            Logger.recordOutput("Roller/CoralRoller/currentCommand", this.getCurrentCommand().getName());
        } else {
            Logger.recordOutput("Roller/CoralRoller/currentCommand", "null");
        }
    }
}
