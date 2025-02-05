package frc.robot.subsystems.roller.sim;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Robot;
import frc.robot.subsystems.roller.PhotoelectricIO;

public class PhotoelectricSim implements PhotoelectricIO { 
    private double scoreTimer = -1;
    private boolean sensing;

    /**
     * Should only be used for simulation implementation
     * @param initialState 
     */
    @Override
    public void setInitialState(boolean initialState) {
        if (initialState) {
            sensing = true;
        } else {
            sensing = false;
        }
    }

    @Override
    public void updateData(PhotoelectricData data) {
        if (Robot.scoringRoller.getCurrentCommand() != null) {
            System.out.println("getCurrentCommand is not null");
            if (scoreTimer == 999999999) { // Initialize timer only once per command
                System.out.println("get the scoreTimer AGAIN");
                scoreTimer = -1;
            }
            
            if (scoreTimer < 0) {  // Initialize timer only once per command
                System.out.println("get the scoreTimer");
                scoreTimer = Timer.getFPGATimestamp();
            }
    
            
            switch (Robot.scoringRoller.getCurrentCommand().getName()) {
                case "Handoff": 
                case "IntakeFloor": 
                case "IntakeSource":
                case "CoralIntakeSource":
                    System.out.println("sensing before score timer" + sensing);
                    // sensing = false;
                    if (Timer.getFPGATimestamp() - scoreTimer > 2) {
                        sensing = true;
                        System.out.println("scoreTimer" + scoreTimer);
                        if (scoreTimer != -1) {
                            scoreTimer = 999999999;
                            System.out.println("sensing set after score timer: " + sensing);
                        } 
                    }
                    break;

                case "ScoreL1": 
                case "OuttakeCoral":
                        // sensing = true; 
                        if (Timer.getFPGATimestamp() - scoreTimer > 2) {
                            sensing = false;
                            if (scoreTimer != -1) {
                                scoreTimer = 999999999; 
                            }
                        }
                    break;
                case "ScoreL234": 
                    // sensing = true;
                    if (Timer.getFPGATimestamp() - scoreTimer > 2) {
                        sensing = false;
                        if (scoreTimer != -1) {
                            System.out.println("sensing change after 2 sec");
                            scoreTimer = 999999999; 
                        } 
                    }
                    break;
            }
        }
        data.sensing = sensing;
    }
}

        

