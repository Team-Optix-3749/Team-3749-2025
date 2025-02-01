package frc.robot.subsystems.arm.coral;

import edu.wpi.first.math.util.Units;

/**
 * Constants file for the climb arm subsystem
 * 
 * @author Weston Gardner
 * 
 */

public class CoralArmConstants {
    public static final int motorId = 0;
    public static final int numMotors = 1;

    public static final double armLength_inches = 17.796; // from cad max length of arm
    public static final double armLength_meters = Units.inchesToMeters(armLength_inches);

    public static final int armMinAngle_degrees = 10;
    public static final int armMaxAngle_degrees = 180;
    public static final int armStartingAngle_degrees = 90;

    public static final double armMass_kg = 1.132; // from cad, I highlighted all of the components on the arm and used the mass feature
    public static final double armGearing = 40;

    public static double kP = 2;
    public static final double kI = 0.0;
    public static final double kD = 0.0;

    public static double kG = 2.90811;
    public static final double kS = 0.0;
    public static final double kA = 0.0;
    public static final double kV = 0.0;

    public static final double stowSetPoint_rad = 70 * Math.PI / 180; // 70, 1.2217304764
    public static final double handOffSetPoint_rad = 40 * Math.PI / 180; // 40
    public static final double coralPickUpSetPoint_rad = 175 * Math.PI / 180; // 175, 3.05432619099
    public static final double L1SetPoint_rad = 180 * Math.PI / 180; // placeholder

    public static final double maxVelocity = 0;
    public static final double maxAcceleration = 0;

    public static final double momentOfInertia = 0.775; // from last years MOI
    public static final boolean simulateGravity = true;

    public enum ArmStates {
        MOVING_DOWN,
        MOVING_UP,
        CORAL_PICKUP,
        L1,
        HAND_OFF,
        STOWED,
        STOPPED
    }
}
