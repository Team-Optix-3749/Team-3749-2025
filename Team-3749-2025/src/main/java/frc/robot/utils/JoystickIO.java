package frc.robot.utils;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Robot;
import frc.robot.commands.example.ExampleSubsystemCommand;
import frc.robot.commands.swerve.DriveStraight;
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
    private static final Command sample = new ExampleSubsystemCommand();
    private static final Command DriveStraight = new DriveStraight();

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
        pilot.a().whileTrue(DriveStraight);

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
                                () -> {
                                    if (pilot.y().getAsBoolean()) {
                                        return 1.0;
                                    }
                                    return 0.0;
                                }));
    }

}
