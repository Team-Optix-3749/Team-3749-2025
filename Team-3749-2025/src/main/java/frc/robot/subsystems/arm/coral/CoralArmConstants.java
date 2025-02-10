package frc.robot.subsystems.arm.coral;


/**
 * Constants file for the climb arm subsystem
 *
 * @author Weston Gardner
 */
public class CoralArmConstants {

	// motor specifications
	public static final int motorID = 16;
	public static final int numMotors = 1;

	// arm specifications
	public static final double armLength_meters = 0.442493;

	public static final int armMinAngle_degrees = 45;
	public static final int armMaxAngle_degrees = 240;
	public static final int armStartingAngle_degrees = 45;

	public static final double armMass_kg = 2.10899999570037;
	public static final double armGearing = 40;

	public static final double maxVelocity = 5;
	public static final double maxAcceleration = 4;

	public static final double momentOfInertia = 0.775;

	// control values
	public static final double kP = 2;
	public static final double kI = 0.0;
	public static final double kD = 0.0;

	public static final double kG = 1.454056;
	public static final double kS = 0.0;
	public static final double kA = 0.0;
	public static final double kV = 1.5;

	// set points
	public static final double stowSetPoint_rad = 45 * Math.PI / 180;
	public static final double handOffSetPoint_rad = 45 * Math.PI / 180;
	public static final double coralPickUpSetPoint_rad = 240 * Math.PI / 180;
	
	// extra
	public static final boolean simulateGravity = true;
	public static final double stateMarginOfError = 0.01;

	public enum ArmStates {
		CORAL_PICKUP,
		HAND_OFF,
		STOWED,
		STOPPED
	}
}
