package frc.robot.subsystems.roller.sim;

import javax.sound.sampled.SourceDataLine;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Robot;
import frc.robot.commands.integration.CoralIntakeSource;
import frc.robot.commands.integration.Handoff;
import frc.robot.commands.integration.IntakeFloor;
import frc.robot.commands.integration.IntakeSource;
import frc.robot.commands.integration.KnockAlgae;
import frc.robot.commands.integration.OuttakeCoral;
import frc.robot.commands.integration.ScoreL1;
import frc.robot.commands.integration.ScoreL234;
import frc.robot.subsystems.roller.PhotoelectricIO;

public class PhotoelectricSim implements PhotoelectricIO { 
    private double scoreTimer = -1;
    private boolean sensing;
    // private boolean changedSensing;
    /**
     * Should only be used for simulation implementation
     * @param initialState 
     */
    @Override
    public void setInitialState(boolean initialState) {
        sensing = initialState;
    }

    /**
     * Updates the photoelectric data with the current state of the roller.
     * 
     * @param data the data to be updated
     */
    @Override
    public void updateData(PhotoelectricData data) {
       // Check if ScoreL234 is currently active
       if (ScoreL234.activeScoreCommand != null) {
      

        if (scoreTimer < 0) {  // Start timer once when command starts
            scoreTimer = Timer.getFPGATimestamp();
        }

        // After 2 seconds, sensing should change
        if (Timer.getFPGATimestamp() - scoreTimer > 2) {
            sensing = false;  // Set sensing to false after 2 seconds
            scoreTimer = -1;  // Reset timer
        }
    }

       // Check if IntakeSource is currently active
       if (IntakeSource.activeIntakeSourceCommand != null) {

        if (scoreTimer < 0) {  // Start timer once when command starts
            scoreTimer = Timer.getFPGATimestamp();
            sensing = false;
        }

        // After 2 seconds, sensing should change
        if (Timer.getFPGATimestamp() - scoreTimer > 2) {
            sensing = true;  // Set sensing to false after 2 seconds
            scoreTimer = -1;  // Reset timer
        
        }
    }

    if (CoralIntakeSource.activeCoralIntakeSourceCommand != null) {

        if (scoreTimer < 0) {  // Start timer once when command starts
            scoreTimer = Timer.getFPGATimestamp();
            sensing = false;
        }

        // After 2 seconds, sensing should change
        if (Timer.getFPGATimestamp() - scoreTimer > 2) {
            sensing = true;  // Set sensing to false after 2 seconds
            scoreTimer = -1;  // Reset timer
        
        }
    }

        // Check if ScoreL1 is currently active
        if (ScoreL1.activeScore1Command != null) {

        if (scoreTimer < 0) {  // Start timer once when command starts
            scoreTimer = Timer.getFPGATimestamp();
            sensing = true;
        }
    
        // After 2 seconds, sensing should change
        if (Timer.getFPGATimestamp() - scoreTimer > 2) {
            sensing = false;  // Set sensing to false after 2 seconds
            scoreTimer = -1;  // Reset timer
            
        }
    }
    // Check if IntakeFloor is currently active
    if (IntakeFloor.activeIntakeFloorCommand != null) {

        if (scoreTimer < 0) {  // Start timer once when command starts
            scoreTimer = Timer.getFPGATimestamp();
            sensing = true;
        }
    
        // After 2 seconds, sensing should change
        if (Timer.getFPGATimestamp() - scoreTimer > 2) {
            sensing = false;  // Set sensing to false after 2 seconds
            scoreTimer = -1;  // Reset timer
            
        }
    }

    data.sensing = sensing;
  
    }
}
    



