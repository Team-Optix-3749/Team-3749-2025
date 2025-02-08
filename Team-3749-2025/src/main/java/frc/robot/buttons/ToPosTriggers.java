package frc.robot.buttons;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.buttons.ButtonBoard.ScoringLocation;
import frc.robot.commands.elevator.SetElevatorState;
import frc.robot.commands.integration.KnockAlgae;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorStates;
import frc.robot.subsystems.swerve.ToPosConstants;
import frc.robot.subsystems.swerve.ToPosConstants.Setpoints.PPSetpoints;
import frc.robot.utils.UtilityFunctions;

public class ToPosTriggers {

    public static BooleanSupplier isReefSupplier = () -> Robot.swerve.getPPSetpoint() == PPSetpoints.A ||
                Robot.swerve.getPPSetpoint() == PPSetpoints.B ||
                Robot.swerve.getPPSetpoint() == PPSetpoints.C ||
                Robot.swerve.getPPSetpoint() == PPSetpoints.D ||
                Robot.swerve.getPPSetpoint() == PPSetpoints.E ||
                Robot.swerve.getPPSetpoint() == PPSetpoints.F ||
                Robot.swerve.getPPSetpoint() == PPSetpoints.G ||
                Robot.swerve.getPPSetpoint() == PPSetpoints.H ||
                Robot.swerve.getPPSetpoint() == PPSetpoints.I ||
                Robot.swerve.getPPSetpoint() == PPSetpoints.J ||
                Robot.swerve.getPPSetpoint() == PPSetpoints.K ||
                Robot.swerve.getPPSetpoint() == PPSetpoints.L;

    public static BooleanSupplier isReefL1Supplier = () -> Robot.swerve.getPPSetpoint() == PPSetpoints.AL1 ||
                Robot.swerve.getPPSetpoint() == PPSetpoints.BL1 ||
                Robot.swerve.getPPSetpoint() == PPSetpoints.CL1 ||
                Robot.swerve.getPPSetpoint() == PPSetpoints.DL1 ||
                Robot.swerve.getPPSetpoint() == PPSetpoints.EL1 ||
                Robot.swerve.getPPSetpoint() == PPSetpoints.FL1 ||
                Robot.swerve.getPPSetpoint() == PPSetpoints.GL1 ||
                Robot.swerve.getPPSetpoint() == PPSetpoints.HL1 ||
                Robot.swerve.getPPSetpoint() == PPSetpoints.IL1 ||
                Robot.swerve.getPPSetpoint() == PPSetpoints.JL1 ||
                Robot.swerve.getPPSetpoint() == PPSetpoints.KL1 ||
                Robot.swerve.getPPSetpoint() == PPSetpoints.LL1; 

    public static BooleanSupplier isCoralSupplier = () -> 
    Robot.swerve.getPPSetpoint() == PPSetpoints.CORALLEFT || 
    Robot.swerve.getPPSetpoint() == PPSetpoints.CORALRIGHT;

    public static BooleanSupplier isHighAlgaeSupplier = () -> 
    Robot.swerve.getPPSetpoint() == PPSetpoints.REEFCLOSE || 
    Robot.swerve.getPPSetpoint() == PPSetpoints.REEFFARLEFT || 
    Robot.swerve.getPPSetpoint() == PPSetpoints.REEFFARRIGHT;

    public static BooleanSupplier isLowAlgaeSupplier = () -> 
    Robot.swerve.getPPSetpoint() == PPSetpoints.REEFFAR || 
    Robot.swerve.getPPSetpoint() == PPSetpoints.REEFCLOSELEFT || 
    Robot.swerve.getPPSetpoint() == PPSetpoints.REEFCLOSERIGHT;

    //all of these suppliers are js, if these conditions are true basically



        private static boolean OTFWithinMargin() {
            return UtilityFunctions.withinMargin(ToPosConstants.Setpoints.approachPointDistance,
                    Robot.swerve.getPose().getTranslation(),
                    Robot.swerve.getPPSetpoint().setpoint.getTranslation()); //if our bot is within a certain distance to the setpoint
        }
    
        public static void createOTFTriggers() {

            //Trigger trigger(boolean condition)
            //trigger.onTrue(command) (the condition is met, usually being within a certain distance to the selected point), and the trigger runs
    
            Trigger coralStation = new Trigger(() -> Robot.swerve.getIsOTF()).and(() -> {
                Boolean withinMargin = OTFWithinMargin();
                return withinMargin && isCoralSupplier.getAsBoolean();
            });
            coralStation.onTrue(new PrintCommand("Coral Station"));
    
            Trigger coralReefL1 = new Trigger(() -> Robot.swerve.getIsOTF()).and(() -> {
                Boolean withinMargin = OTFWithinMargin();
                Boolean isCoralReef = isReefL1Supplier.getAsBoolean();
            Boolean isL1 = JoystickIO.buttonBoard.getScoringLocation() == ScoringLocation.L1;
            return withinMargin && isCoralReef && isL1;
        });
        coralReefL1.onTrue(new SetElevatorState(ElevatorStates.L1)); 
        

        Trigger coralReefL2 = new Trigger(() -> Robot.swerve.getIsOTF()).and(() -> {
            Boolean withinMargin = OTFWithinMargin();
            // System.out.println(withinMargin);
            Boolean isCoralReef = isReefSupplier.getAsBoolean();
            Boolean isL2 = JoystickIO.buttonBoard.getScoringLocation() == ScoringLocation.L2;
            return withinMargin && isCoralReef && isL2;
        });
        coralReefL2.onTrue(new SetElevatorState(ElevatorStates.L2));

        Trigger coralReefL3 = new Trigger(() -> Robot.swerve.getIsOTF()).and(() -> {
            Boolean withinMargin = OTFWithinMargin();
            Boolean isCoralReef = isReefSupplier.getAsBoolean();
            Boolean isL3 = JoystickIO.buttonBoard.getScoringLocation() == ScoringLocation.L3;
            return withinMargin && isCoralReef && isL3;
        });
        coralReefL3.onTrue(new SetElevatorState(ElevatorStates.L3));

        Trigger coralReefL4 = new Trigger(() -> Robot.swerve.getIsOTF()).and(() -> {
            Boolean withinMargin = OTFWithinMargin();
            Boolean isCoralReef = isReefSupplier.getAsBoolean();
            Boolean isL4 = JoystickIO.buttonBoard.getScoringLocation() == ScoringLocation.L4;
            return withinMargin && isCoralReef && isL4;
        });
        coralReefL4.onTrue(new SetElevatorState(ElevatorStates.L4));

        Trigger highAlgaeTrigger = new Trigger(() -> Robot.swerve.getIsOTF()).and(() -> {
            Boolean withinMargin = OTFWithinMargin();
            Boolean isAlgaeKnocking = isHighAlgaeSupplier.getAsBoolean();
            return withinMargin && isAlgaeKnocking;
        });
        highAlgaeTrigger.onTrue(new KnockAlgae(ElevatorStates.ALGAE_HIGH));

        Trigger lowAlgaeTrigger = new Trigger(() -> Robot.swerve.getIsOTF()).and(() -> {
            Boolean withinMargin = OTFWithinMargin();
            Boolean isAlgaeKnocking = isLowAlgaeSupplier.getAsBoolean();
            return withinMargin && isAlgaeKnocking;
        });
        lowAlgaeTrigger.onTrue(new KnockAlgae(ElevatorStates.ALGAE_LOW));

    }
}
