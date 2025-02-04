package frc.robot.utils;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.commands.example.ExampleSubsystemCommand;
import frc.robot.commands.swerve.DriveStraight;
import frc.robot.commands.swerve.OnTheFly;
import frc.robot.commands.swerve.SwerveDefaultCommand;

/**
 * Util class for button bindings
 * 
 * @author Rohin Sood
 * @author Noah Simon
 */
public class JoystickIO {

    private static final CommandXboxController pilot = new CommandXboxController(0);
    private static final CommandXboxController operator = new CommandXboxController(1);
    //private static final Command sample = new ExampleSubsystemCommand(); it was getting on my nerves seeing the warning
    private static final Command driveStraight = new DriveStraight();
    private static final Command onTheFly = new OnTheFly();
    
    // private static final SetElevatorState l1 = new SetElevatorState(ElevatorStates.L1);
    // private static final SetElevatorState l2 = new SetElevatorState(ElevatorStates.L2);
    // private static final SetElevatorState l3 = new SetElevatorState(ElevatorStates.L3);
    // private static final SetElevatorState l4 = new SetElevatorState(ElevatorStates.L4);

    private final static GenericHID buttonBoardPlayer1 = new GenericHID(1);
    private final static GenericHID buttonBoardPlayer2 = new GenericHID(2);

    // coral elevator positions
    private final static JoystickButton buttonl1 = new JoystickButton(buttonBoardPlayer1, 1);
    private final static JoystickButton buttonl2 = new JoystickButton(buttonBoardPlayer1, 2);
    private final static JoystickButton buttonl3 = new JoystickButton(buttonBoardPlayer1, 3);
    private final static JoystickButton buttonl4 = new JoystickButton(buttonBoardPlayer1, 4);

    // source buttons
    private final static JoystickButton buttonRightSource = new JoystickButton(buttonBoardPlayer1, 5);
    private final static JoystickButton buttonLeftSource = new JoystickButton(buttonBoardPlayer1, 6);

    // reef positions
    private final static JoystickButton buttonReefZoneA = new JoystickButton(buttonBoardPlayer2, 7);
    private final static JoystickButton buttonReefZoneB = new JoystickButton(buttonBoardPlayer2, 8);
    private final static JoystickButton buttonReefZoneC = new JoystickButton(buttonBoardPlayer2, 9);
    private final static JoystickButton buttonReefZoneD = new JoystickButton(buttonBoardPlayer2, 10);
    private final static JoystickButton buttonReefZoneE = new JoystickButton(buttonBoardPlayer2, 11);
    private final static JoystickButton buttonReefZoneF = new JoystickButton(buttonBoardPlayer2, 12);
    private final static JoystickButton buttonReefZoneG = new JoystickButton(buttonBoardPlayer2, 13);
    private final static JoystickButton buttonReefZoneH = new JoystickButton(buttonBoardPlayer2, 14);
    private final static JoystickButton buttonReefZoneI = new JoystickButton(buttonBoardPlayer2, 15);
    private final static JoystickButton buttonReefZoneJ = new JoystickButton(buttonBoardPlayer2, 16);
    private final static JoystickButton buttonReefZoneK = new JoystickButton(buttonBoardPlayer2, 17);
    private final static JoystickButton buttonReefZoneL = new JoystickButton(buttonBoardPlayer2, 18);

    // miscellaneous buttons
    private final static JoystickButton buttonAlgaeKnockoff = new JoystickButton(buttonBoardPlayer1, 19);
    private final static JoystickButton buttonUtilityA = new JoystickButton(buttonBoardPlayer1, 20);
    private final static JoystickButton buttonUtilityB = new JoystickButton(buttonBoardPlayer1, 21);

    public JoystickIO() {
    }

    /**
     * Calls binding methods according to the joysticks connected
     */
    public static void getButtonBindings() {

        if (DriverStation.isJoystickConnected(1)) {
            // if both xbox controllers are connected
            pilotAndOperatorBindings();
        } else if (DriverStation.isJoystickConnected(0)) {
            // if only one xbox controller is connected
            pilotBindings();
        } else if (Robot.isSimulation()) {
            // will show not connected if on
            pilotAndOperatorBindings();
            // simBindings();
        } else {

        }

        setDefaultCommands();
    }

    /**
     * If both controllers are plugged in (pi and op)
     */
    public static void pilotAndOperatorBindings() {
        // gyro reset
        pilot.start().onTrue(Commands.runOnce(() -> Robot.swerve.resetGyro()));
        pilot.a().whileTrue(driveStraight);

        pilot.x().onTrue(Commands.runOnce(()->{Robot.swerve.isOTF=true;}));

        new Trigger(()->Robot.swerve.isOTF).whileTrue(onTheFly);

        pilot.b().onTrue(Commands.runOnce(() -> {
            Robot.swerve.isOTF = false;
            Robot.swerve.cyclePPSetpoint();
            Robot.swerve.showSetpointEndGoal();
        }));

        buttonReefZoneA.onTrue(Commands.runOnce(() -> Robot.swerve.startOnTheFly(5)));
        buttonReefZoneB.onTrue(Commands.runOnce(() -> Robot.swerve.startOnTheFly(7)));
        buttonReefZoneC.onTrue(Commands.runOnce(() -> Robot.swerve.startOnTheFly(9)));
        buttonReefZoneD.onTrue(Commands.runOnce(() -> Robot.swerve.startOnTheFly(11)));
        buttonReefZoneE.onTrue(Commands.runOnce(() -> Robot.swerve.startOnTheFly(13)));
        buttonReefZoneF.onTrue(Commands.runOnce(() -> Robot.swerve.startOnTheFly(15)));
        buttonReefZoneG.onTrue(Commands.runOnce(() -> Robot.swerve.startOnTheFly(17)));
        buttonReefZoneH.onTrue(Commands.runOnce(() -> Robot.swerve.startOnTheFly(19)));
        buttonReefZoneI.onTrue(Commands.runOnce(() -> Robot.swerve.startOnTheFly(21)));
        buttonReefZoneJ.onTrue(Commands.runOnce(() -> Robot.swerve.startOnTheFly(23)));
        buttonReefZoneK.onTrue(Commands.runOnce(() -> Robot.swerve.startOnTheFly(25)));
        buttonReefZoneL.onTrue(Commands.runOnce(() -> Robot.swerve.startOnTheFly(27)));

        // Example binding
        operator.a().whileTrue(new ExampleSubsystemCommand());

    }
    public static void pilotBindings() {
        // gyro reset
        pilot.start().onTrue(Commands.runOnce(() -> Robot.swerve.resetGyro()));

        // Example binding
        pilot.a().whileTrue(new ExampleSubsystemCommand());
    }

    public static void simBindings() {
        pilotBindings();
    }

    /**
     * Sets the default commands
     */
    public static void setDefaultCommands() {
        if (Robot.isSimulation()) {
            setSimDefaultCommands();
        } else {
            setRealDefaultCommands();
        }
    }

    private static void setRealDefaultCommands() {
        Robot.swerve.setDefaultCommand(
                new SwerveDefaultCommand(
                        () -> pilot.getLeftX(),
                        () -> pilot.getLeftY(),
                        () -> pilot.getRightX()));
    }

    private static void setSimDefaultCommands() {
        Robot.swerve
                .setDefaultCommand(
                        new SwerveDefaultCommand(
                                () -> pilot.getLeftX(),
                                () -> pilot.getLeftY(),
                                () -> pilot.getRightX()
                                ));
    }

}
