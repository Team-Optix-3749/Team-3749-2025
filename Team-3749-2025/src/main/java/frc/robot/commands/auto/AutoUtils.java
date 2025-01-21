package frc.robot.commands.auto;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoTrajectory;
import choreo.auto.AutoRoutine;
import choreo.util.ChoreoAllianceFlipUtil;
import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Robot;

/**
 * All setup and helper methods for auto routines, including the
 * choreo factory and auto selector
 * 
 * @author Noah Simon
 */
public class AutoUtils {
    // make sure to properly log the robot's setpoints

    private static AutoFactory factory;
    private static AutoChooser chooser;

    public static ChoreoAllianceFlipUtil.Flipper flipper = ChoreoAllianceFlipUtil.getFlipper();

    /**
     * run all necessary auto setup methods
     */
    public static void initAuto() {
        setupFactory(true);
        setupChooser();
    }

    /**
     * 
     * @return the auto selector object
     */
    public static AutoChooser getChooser() {
        return chooser;
    }

    public static AutoFactory getAutoFactory() {
        return factory;
    }

    /**
     * setup the choreo factor object with bindings, controller, etc.
     */
    private static void setupFactory(boolean isFlipped) {
        /**
         * Swerve Pose Supplier
         * Reset Odometry Method
         * Swerve Controller (PID and set chassis speeds)
         * Alliance Supplier for swapping
         * Swerve Subsystem for scheduling
         * Bindings, created above
         */

        // will now take a reset odometry

        factory = new AutoFactory(() -> Robot.swerve.getPose(),
                (Pose2d startingPose) -> Robot.swerve
                        .setOdometry(isFlipped ? getXFlippedPose(startingPose) : startingPose),
                (SwerveSample sample) -> Robot.swerve.followSample(sample, isFlipped),
                true,
                Robot.swerve);
        // Event Binding
        factory.bind("Marker", Commands.print("Marker Passed"));

    }

    /**
     * setup the choreo auto chooser and assign it to the Shuffleboard/Auto tab of
     * networktables
     */

    private static void setupChooser() {
        // interface for choreo

        // Made sendable, use SmartDashbaord now
        chooser = new AutoChooser();
        chooser.addCmd("My Routine", () -> Autos.getMyRoutine());
        chooser.addCmd("Print", () -> Autos.getPrint());
        chooser.addCmd("Split", () -> Autos.getSplitRoutine());
        chooser.addCmd("Straight", () -> Autos.getStraight());
        chooser.addCmd("Chair", () -> Autos.getChairGame());
        chooser.addCmd("Score/pick note", () -> Autos.getScore_Pick());
        chooser.addCmd("Team Taxi", () -> Autos.getTeamTaxi());
        chooser.addCmd("Push Right and Taxi", () -> Autos.getPushRightAndTaxi());
        chooser.addCmd("Push Left and Taxi", () -> Autos.getPushLeftAndTaxi());
        chooser.addCmd("3 Piece", () -> Autos.get3Piece());
        chooser.addCmd("1 Piece", () -> Autos.get1Piece());
        // Default
        chooser.select("Straight");

        SmartDashboard.putData("Auto: Auto Chooser", chooser);

    }

    public static Command getSingleTrajectory(String trajectoryName) {
        AutoRoutine routine = factory.newRoutine(trajectoryName);
        AutoTrajectory trajectory = routine.trajectory(trajectoryName);

        Command trajectoryCommand = trajectory.cmd();

        routine.active().onTrue(
                factory.resetOdometry(trajectoryName).andThen(
                        trajectoryCommand));

        System.out.println(trajectory.getInitialPose().get());
        return Commands.print(trajectoryName).andThen(routine.cmd());

    }

    /**
     * Each pos will be flipped across the X-axis using Choreo's Flip Util
     * Returns new pos
     * Used in auto factory for flipped paths
     */
    private static Pose2d getXFlippedPose(Pose2d pos) {

        double newX = pos.getX();
        double newY = flipper.flipY(pos.getY());
        double newHeading = pos.getRotation().getRadians() * -1;

        return new Pose2d(newX, newY,
                new Rotation2d(newHeading));
    }

}
