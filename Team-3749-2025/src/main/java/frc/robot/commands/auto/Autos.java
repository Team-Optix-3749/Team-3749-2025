package frc.robot.commands.auto;

import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

/**
 * Class containing our auto routines. Referenced by the auto selector and
 * potentially robot container
 * 
 * @author Noah Simon
 */
public class Autos {

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

   
    /**
     * 
     * @return a command that scores at positions 5, 4, and 3 at L4 
     */

    public static Command get3Piece() {
        AutoRoutine routine = AutoUtils.getAutoFactory().newRoutine("3-Piece");

        // loop.trajectory, or the new name
        AutoTrajectory trajectory1 = routine.trajectory("Start-L5");
        AutoTrajectory trajectory2 = routine.trajectory("L5-Station");
        AutoTrajectory trajectory3 = routine.trajectory("Station-L4");
        AutoTrajectory trajectory4 = routine.trajectory("L4-Station");
        AutoTrajectory trajectory5 = routine.trajectory("Station-L3");

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

        return Commands.print("3 piece auto!").andThen(
                AutoUtils.startRoutine(routine, "Start-L5", trajectory1));
    }

     /**
     * 
     * @return a command that Taxi's to L5
     */

    public static Command getTaxi() {
        AutoRoutine routine = AutoUtils.getAutoFactory().newRoutine("Taxi");

        // loop.trajectory, or the new name
        AutoTrajectory trajectory1 = routine.trajectory("Start-L5");
     
        return Commands.print("Taxi").andThen(
                AutoUtils.startRoutine(routine, "Start-L5", trajectory1));
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
     * @return a command that goes to middle to score(used when in middle)
     */

    public static Command getOnePieceCenter() {
        AutoRoutine routine = AutoUtils.getAutoFactory().newRoutine("One Piece Center");

        // loop.trajectory, or the new name
        AutoTrajectory trajectory1 = routine.trajectory("MidStart-L6");
       

        return Commands.print("One Piece Center").andThen(
                AutoUtils.startRoutine(routine, "MidStart-L6", trajectory1));
    }

      /**
     * 
     * @return a command that scores at positions 5, 4, 3, and 2 at L4 
     */

    public static Command get4Piece() {
        AutoRoutine routine = AutoUtils.getAutoFactory().newRoutine("4-Piece Coral");

        // loop.trajectory, or the new name
        AutoTrajectory trajectory1 = routine.trajectory("Start-L5");
        AutoTrajectory trajectory2 = routine.trajectory("L5-Station");
        AutoTrajectory trajectory3 = routine.trajectory("Station-L4");
        AutoTrajectory trajectory4 = routine.trajectory("L4-Station");
        AutoTrajectory trajectory5 = routine.trajectory("Station-L3");
        AutoTrajectory trajectory6 = routine.trajectory("L3-Station");
        AutoTrajectory trajectory7 = routine.trajectory("Station-L2");

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

        return Commands.print("4 piece auto!").andThen(
                AutoUtils.startRoutine(routine, "Start-L5", trajectory1));
    }

      /**
     * 
     * @return a command that scores at positions 5, 4, and 3 at L4. Then, it goes to knock algae at 2, then it scores at 2 and finally Knocks Algae at 1. 
     */

    public static Command get3CoralAnd2Algae() {
        AutoRoutine routine = AutoUtils.getAutoFactory().newRoutine("3 Coral and 2 Algae");

        // loop.trajectory, or the new name
        AutoTrajectory trajectory1 = routine.trajectory("Start-L5");
        AutoTrajectory trajectory2 = routine.trajectory("L5-Station");
        AutoTrajectory trajectory3 = routine.trajectory("Station-L4");
        AutoTrajectory trajectory4 = routine.trajectory("L4-Station");
        AutoTrajectory trajectory5 = routine.trajectory("Station-L3");
        AutoTrajectory trajectory6 = routine.trajectory("L3-KnockAlgae2");
        AutoTrajectory trajectory7 = routine.trajectory("KnockAlgae2-Station");
        AutoTrajectory trajectory8 = routine.trajectory("Station-L2");
        AutoTrajectory trajectory9 = routine.trajectory("L2-KnockAlgae1");


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
        System.out.println("7");
        AutoUtils.goNextAfterCommand(trajectory7, trajectory8, intake3);
        System.out.println("6");
        AutoUtils.goNextAfterCommand(trajectory6, trajectory7, knockalgae1);
        System.out.println("5");
        AutoUtils.goNextAfterCommand(trajectory5, trajectory6, score3);
        System.out.println("4");
        AutoUtils.goNextAfterCommand(trajectory4, trajectory5, intake2);
        System.out.println("3");
        AutoUtils.goNextAfterCommand(trajectory3, trajectory4, score2);
        System.out.println("2");
        AutoUtils.goNextAfterCommand(trajectory2, trajectory3, intake1);
        System.out.println("1");
        AutoUtils.goNextAfterCommand(trajectory1, trajectory2, score1);

        return Commands.print("3 Coral and 2 Algae").andThen(
                AutoUtils.startRoutine(routine, "Start-L5", trajectory1));
    }

    public static Command get3CoralScoreL1() {
        AutoRoutine routine = AutoUtils.getAutoFactory().newRoutine("3 Coral Score L1");

        AutoTrajectory trajectory1 = routine.trajectory("Start-L5");
        AutoTrajectory trajectory2 = routine.trajectory("L5-Station");
        AutoTrajectory trajectory3 = routine.trajectory("Station-L4");
        AutoTrajectory trajectory4 = routine.trajectory("L4-Station");
        AutoTrajectory trajectory5 = routine.trajectory("Station-L3");
        AutoTrajectory trajectory6 = routine.trajectory("L3-KnockAlgae2");
        AutoTrajectory trajectory7 = routine.trajectory("KnockAlgae2-Station");

        Command score1 = AutoUtils.addScoreL1(trajectory1);
        Command intake1 = AutoUtils.addIntakeCoralArm(trajectory2);
        Command score2 = AutoUtils.addScoreL1(trajectory3);
        Command intake2 = AutoUtils.addIntakeCoralArm(trajectory4);
        Command score3 = AutoUtils.addScoreL1(trajectory5); 
        Command intake3 = AutoUtils.addIntakeCoralArm(trajectory7);

        AutoUtils.goNextAfterCommand(trajectory6, trajectory7, intake3);
        AutoUtils.goNextAfterCommand(trajectory5, trajectory6, score3);
        AutoUtils.goNextAfterCommand(trajectory4, trajectory5, intake2);
        AutoUtils.goNextAfterCommand(trajectory3, trajectory4, score2);
        AutoUtils.goNextAfterCommand(trajectory2, trajectory3, intake1);
        AutoUtils.goNextAfterCommand(trajectory1, trajectory2, score1);

        return Commands.print("3 Coral Score L1").andThen(
            AutoUtils.startRoutine(routine, "Start-L5", trajectory1));
    }
}