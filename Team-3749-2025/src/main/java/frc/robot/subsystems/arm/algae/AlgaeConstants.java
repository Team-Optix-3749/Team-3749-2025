package frc.robot.subsystems.arm.algae;

import edu.wpi.first.math.util.Units;

/**
 * Constants file for the algae arm subsystem
 * 
 * @author Weston Gardner
 * 
 */

public class AlgaeConstants {

    public static final int motorId = 0;
    public static final int numMotors = 1;

    public static final double armLength_inches = 25.47; // from cad
    public static final double armLength_meters = Units.inchesToMeters(armLength_inches);

    public static final int armMinAngle_degrees = 80;
    public static final int armMaxAngle_degrees = 210;
    public static final int armStartingAngle_degrees = 90;

    public static final double armMass_kg = 1.5952844;
    public static final double armGearing = 49; // rough estimate

    public static double kP = 4;
    public static final double kI = 0.0;
    public static final double kD = 0.0;

    public static double kG = 2.48513;
    public static final double kS = 0.0;
    public static final double kA = 0.0;
    public static final double kV = 0.0;

    public static final double stowSetPoint_rad = 80 * Math.PI / 180; // 80 degrees
    public static final double processorSetPoint_rad = Math.PI / 2; // 90 degrees
    public static final double algaePickUpSetPoint_rad = 160 * Math.PI / 180; // 160 degrees

    public static final double maxVelocity = 0.0;
    public static final double maxAcceleration = 0.0;

    public static final double momentOfInertia = 57.929; // from cad
    public static final boolean simulateGravity = true;

    public enum ArmStates {
        MOVING_DOWN,
        MOVING_UP,
        STOWED,
        PROCESSOR,
        ALGAE_PICKUP,
        STOPPED
    }
}
