package frc.robot.commands.auto;

import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * Class containing our auto routines. Referenced by the auto selector and
 * potentially robot container
 * 
 * @author Noah Simon
 */
public class Autos {
    private static boolean routineStarted = false; 

    /***
     * 
     * @return The command current selected by the auto chooser
     */
    public static Command getSelectedCommand() {
        return AutoUtils.getChooser().selectedCommand();
    }

    /***
     * A do nothing auto
     * 
     * @return Print Command
     */
    public static Command getPrint() {
        return Commands.print("Print Auto!");
    }
    
    /***
     * Checks whether routine started
     * 
     * @return routineStarted
     */
    public static boolean isRoutineStarted() {
        return routineStarted;
    }

    /**
     * Has routineStarted as true at the start of a routine
     */
    public static void startRoutineTracking() {
        routineStarted = true;
    }

    /**
     * Has routineStarted as false at the end of a routine
     */
    public static void stopRoutineTracking() {
        routineStarted = false;
    }

    /**
     * 
     * @return a command that scores at positions 5, 4, and 3 at L4 
     */
    public static Command get3Piece() {
        AutoRoutine routine = AutoUtils.getAutoFactory().newRoutine("3-Piece");

        // loop.trajectory, or the new name
        AutoTrajectory trajectory1 = routine.trajectory("Start-5");
        AutoTrajectory trajectory2 = routine.trajectory("5-Station");
        AutoTrajectory trajectory3 = routine.trajectory("Station-4");
        AutoTrajectory trajectory4 = routine.trajectory("4-Station");
        AutoTrajectory trajectory5 = routine.trajectory("Station-3");

        // Commands to scoreL4 and intake from source
        Command score1 = AutoUtils.addScoreL4(trajectory1);
        Command intake1 = AutoUtils.addIntake(trajectory2);
        Command score2 = AutoUtils.addScoreL4(trajectory3);
        Command intake2 = AutoUtils.addIntake(trajectory4);
        AutoUtils.addScoreL4(trajectory5); // third score is the end of the routine, so no need for reference

        // reverse order here (ex. connect 3 to 2, THEN 2 to 1)
        AutoUtils.goNextAfterCommand(trajectory4, trajectory5, intake2);
        AutoUtils.goNextAfterCommand(trajectory3, trajectory4, score2);
        AutoUtils.goNextAfterCommand(trajectory2, trajectory3, intake1);
        AutoUtils.goNextAfterCommand(trajectory1, trajectory2, score1);
        
        // Trigger to update routineStarted when routine ends
        new Trigger(() -> trajectory5.cmd().isFinished())
        .onTrue(Commands.runOnce(() -> stopRoutineTracking()));

        return Commands.print("3 piece auto!")
            .andThen(Commands.runOnce(() -> startRoutineTracking()))  // Track routine start
            .andThen(AutoUtils.startRoutine(routine, "Start-5", trajectory1));
    }

     /**
     * 
     * @return a command that Taxi's to L5
     */
    public static Command getTaxi() {
        AutoRoutine routine = AutoUtils.getAutoFactory().newRoutine("Taxi");

        // loop.trajectory, or the new name
        AutoTrajectory trajectory1 = routine.trajectory("Start-5");
     
        return Commands.print("Taxi").andThen(
                AutoUtils.startRoutine(routine, "Start-5", trajectory1));
    }

    /**
     * 
     * @return a command that scores at positions 5 and 4 at L1
     */
    public static Command getTwoPieceScoreL1(){
        AutoRoutine routine = AutoUtils.getAutoFactory().newRoutine("Two Piece Score L1");

        // loop.trajectory, or the new name
        AutoTrajectory trajectory1 = routine.trajectory("Start-5");
        AutoTrajectory trajectory2 = routine.trajectory("5-Station");
        AutoTrajectory trajectory3 = routine.trajectory("Station-4");
       
        // Commands to scoreL1 and intake from source
        Command score1 = AutoUtils.addScoreL1(trajectory1);
        Command intake1 = AutoUtils.addCoralIntakeSource(trajectory2);
        AutoUtils.addScoreL1(trajectory3); // second score is the end of the routine, so no need for reference

        // reverse order here
        AutoUtils.goNextAfterCommand(trajectory2, trajectory3, intake1);
        AutoUtils.goNextAfterCommand(trajectory1, trajectory2, score1);

        // Trigger to update routineStarted when routine ends
        new Trigger(() -> trajectory3.cmd().isFinished())
          .onTrue(Commands.runOnce(() -> stopRoutineTracking()));

        return Commands.print("Two Piece Score L1")
                .andThen(Commands.runOnce(() -> startRoutineTracking())) // Track routine start
                .andThen(AutoUtils.startRoutine(routine, "Start-5", trajectory1));
    }

    /**
     * 
     * @return a command that Taxi's your teammate
     */
    public static Command getTeamtaxi() {
        AutoRoutine routine = AutoUtils.getAutoFactory().newRoutine("Team Taxi");

        // loop.trajectory, or the new name
        AutoTrajectory trajectory1 = routine.trajectory("Start-TeamTaxi");

        return Commands.print("Team Taxi").andThen(
                AutoUtils.startRoutine(routine, "Start-TeamTaxi", trajectory1));
    }

    /**
     * 
     * @return a command that goes to middle to score (used when in middle)
     */
    public static Command getOnePieceCenter() {
        AutoRoutine routine = AutoUtils.getAutoFactory().newRoutine("One Piece Center");

        // loop.trajectory, or the new name
        AutoTrajectory trajectory1 = routine.trajectory("MidStart-6");
        AutoUtils.addScoreL4(trajectory1); // first score is the end of the routine, so no need for reference

        // Trigger to update routineStarted when routine ends
        new Trigger(() -> trajectory1.cmd().isFinished())
          .onTrue(Commands.runOnce(() -> stopRoutineTracking()));
          
        return Commands.print("One Piece Center")
            .andThen(Commands.runOnce(() -> startRoutineTracking()) // Track routine start
            .andThen(AutoUtils.startRoutine(routine, "MidStart-6", trajectory1)));
    }

    /**
     * 
     * @return a command that scores at positions 5, 4, 3, and 2 at L4 
     */
    public static Command get4Piece() {
        AutoRoutine routine = AutoUtils.getAutoFactory().newRoutine("4-Piece Coral");

        // loop.trajectory, or the new name
        AutoTrajectory trajectory1 = routine.trajectory("Start-5");
        AutoTrajectory trajectory2 = routine.trajectory("5-Station");
        AutoTrajectory trajectory3 = routine.trajectory("Station-4");
        AutoTrajectory trajectory4 = routine.trajectory("4-Station");
        AutoTrajectory trajectory5 = routine.trajectory("Station-3");
        AutoTrajectory trajectory6 = routine.trajectory("3-Station");
        AutoTrajectory trajectory7 = routine.trajectory("Station-2");

        // Commands to scoreL4 and intake from source
        Command score1 = AutoUtils.addScoreL4(trajectory1);
        Command intake1 = AutoUtils.addIntake(trajectory2);
        Command score2 = AutoUtils.addScoreL4(trajectory3);
        Command intake2 = AutoUtils.addIntake(trajectory4);
        Command score3 = AutoUtils.addScoreL4(trajectory5); 
        Command intake3 = AutoUtils.addIntake(trajectory6);
        AutoUtils.addScoreL4(trajectory7); // fourth score is the end of the routine, so no need for reference

        // reverse order here (ex. connect 3 to 2, THEN 2 to 1)
        AutoUtils.goNextAfterCommand(trajectory6, trajectory7, intake3);
        AutoUtils.goNextAfterCommand(trajectory5, trajectory6, score3);
        AutoUtils.goNextAfterCommand(trajectory4, trajectory5, intake2);
        AutoUtils.goNextAfterCommand(trajectory3, trajectory4, score2);
        AutoUtils.goNextAfterCommand(trajectory2, trajectory3, intake1);
        AutoUtils.goNextAfterCommand(trajectory1, trajectory2, score1);

        // Trigger to update routineStarted when routine ends
        new Trigger(() -> trajectory7.cmd().isFinished())
          .onTrue(Commands.runOnce(() -> stopRoutineTracking()));

        return Commands.print("4 piece auto!")
            .andThen(Commands.runOnce(() -> startRoutineTracking())) // Track routine start
            .andThen(AutoUtils.startRoutine(routine, "Start-5", trajectory1));
    }

    /**
     * 
     * @return a command that scores at positions 5, 4, and 3 at L4. Then, it goes to knock algae at 2, then it scores at 2 and finally Knocks Algae at 1. 
     */
    public static Command get3CoralAnd2Algae() {
        AutoRoutine routine = AutoUtils.getAutoFactory().newRoutine("3 Coral and 2 Algae");

        // loop.trajectory, or the new name
        AutoTrajectory trajectory1 = routine.trajectory("Start-5");
        AutoTrajectory trajectory2 = routine.trajectory("5-Station");
        AutoTrajectory trajectory3 = routine.trajectory("Station-4");
        AutoTrajectory trajectory4 = routine.trajectory("4-Station");
        AutoTrajectory trajectory5 = routine.trajectory("Station-3");
        AutoTrajectory trajectory6 = routine.trajectory("3-KnockAlgae2");
        AutoTrajectory trajectory7 = routine.trajectory("KnockAlgae2-Station");
        AutoTrajectory trajectory8 = routine.trajectory("Station-2");
        AutoTrajectory trajectory9 = routine.trajectory("2-KnockAlgae1");

        // Commands to scoreL4, intake from source, and knock algae
        Command score1 = AutoUtils.addScoreL4(trajectory1);
        Command intake1 = AutoUtils.addIntake(trajectory2);
        Command score2 = AutoUtils.addScoreL4(trajectory3);
        Command intake2 = AutoUtils.addIntake(trajectory4);
        Command score3 = AutoUtils.addScoreL4(trajectory5); 
        Command knockalgae1 = AutoUtils.addKnockAlgae(trajectory6);
        Command intake3 = AutoUtils.addIntake(trajectory7);
        Command score4 = AutoUtils.addScoreL4(trajectory8);
        AutoUtils.addKnockAlgae(trajectory9); // final score is the end of the routine, so no need for reference

        // reverse order here (ex. connect 3 to 2, THEN 2 to 1)
        AutoUtils.goNextAfterCommand(trajectory8, trajectory9, score4);
        AutoUtils.goNextAfterCommand(trajectory7, trajectory8, intake3);
        AutoUtils.goNextAfterCommand(trajectory6, trajectory7, knockalgae1);
        AutoUtils.goNextAfterCommand(trajectory5, trajectory6, score3);
        AutoUtils.goNextAfterCommand(trajectory4, trajectory5, intake2);
        AutoUtils.goNextAfterCommand(trajectory3, trajectory4, score2);
        AutoUtils.goNextAfterCommand(trajectory2, trajectory3, intake1);
        AutoUtils.goNextAfterCommand(trajectory1, trajectory2, score1);

        // Trigger to update routineStarted when routine ends
        new Trigger(() -> trajectory9.cmd().isFinished())
          .onTrue(Commands.runOnce(() -> stopRoutineTracking()));

        return Commands.print("3 Coral and 2 Algae")
            .andThen(Commands.runOnce(() -> startRoutineTracking())) // Track routine start
            .andThen(AutoUtils.startRoutine(routine, "Start-5", trajectory1));
    }
}