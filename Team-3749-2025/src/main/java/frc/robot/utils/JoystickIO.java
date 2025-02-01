package frc.robot.utils;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Robot;
import frc.robot.commands.arm.SetClimbArmState;
import frc.robot.commands.arm.SetCoralArmState;
import frc.robot.commands.roller.MaintainCommand;
import frc.robot.commands.roller.RunCommand;
import frc.robot.commands.roller.StopCommand;
import frc.robot.commands.swerve.SwerveDefaultCommand;
import frc.robot.subsystems.arm.coral.CoralConstants;
import frc.robot.subsystems.arm.climb.ClimbConstants;

/**
 * Util class for button bindings
 * 
 * @author Rohin Sood
 * @author Noah Simon
 */
public class JoystickIO {

    private static final CommandXboxController pilot = new CommandXboxController(0);
    private static final CommandXboxController operator = new CommandXboxController(1);
    private static final Command MaintainCommand = new MaintainCommand();
    private static final Command RunCommand = new RunCommand();
    private static final Command StopCommand = new StopCommand();

    private static final Command coralStow = new SetCoralArmState(Robot.coralArm, CoralConstants.ArmStates.STOWED,
            CoralConstants.stowSetPoint_rad);
    private static final Command coralPickUp = new SetCoralArmState(Robot.coralArm, CoralConstants.ArmStates.CORAL_PICKUP,
            CoralConstants.coralPickUpSetPoint_rad);

    private static final Command climbStow = new SetClimbArmState(Robot.climbArm, ClimbConstants.ArmStates.STOWED,
            ClimbConstants.stowSetPoint_rad);
    private static final Command climb = new SetClimbArmState(Robot.climbArm, ClimbConstants.ArmStates.CLIMB,
            ClimbConstants.climbSetPoint_rad);

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
        pilot.a().whileTrue(RunCommand);
        pilot.b().whileTrue(MaintainCommand);
        pilot.x().whileTrue(StopCommand);

        // Example binding
        // operator.a().whileTrue(new ExampleSubsystemCommand());

        // operator.a().onTrue(l1);
        // operator.b().onTrue(l2);
        // operator.x().onTrue(l3);
        // operator.y().onTrue(l4);

        operator.a().onTrue(coralStow);
        operator.b().onTrue(coralPickUp);
        operator.x().onTrue(climbStow);
        operator.y().onTrue(climb);

    }

    public static void pilotBindings() {
        // gyro reset
        pilot.start().onTrue(Commands.runOnce(() -> Robot.swerve.resetGyro()));

        // Example binding
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
