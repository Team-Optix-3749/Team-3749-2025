package frc.robot.subsystems.arm.coral;

import edu.wpi.first.math.util.Units;
import frc.robot.utils.LoggedTunableNumber;

/**
 * Constants file for the climb arm subsystem
 *
 * @author Weston Gardner
 */
public class CoralArmConstants {

	// motor specifications
	public static final int motorID = 16;
	public static final int numMotors = 1;
	public static final double absoluteEncoderOffsetRad = Units.degreesToRadians(39.32905);

	public static final double armLengthIn = 17.796; // from cad max length of arm
	public static final double armLengthMeters = Units.inchesToMeters(armLengthIn);

	public static final int armMinAngleDegrees = 10;
	public static final int armMaxAngleDegrees = 180;
	public static final int armStartingAngleDegrees = 90;

	public static final double armMassKg = 1.132; // from cad, I highlighted all of the components on the arm and used
													// the mass feature
	public static final double armGearing = 45;

	// control values
	public static LoggedTunableNumber kG = new LoggedTunableNumber("Arms/CoralArm/kG", 0.39);
	public static LoggedTunableNumber kP = new LoggedTunableNumber("Arms/CoralArm/kP", 2.2);
	public static LoggedTunableNumber kI = new LoggedTunableNumber("Arms/CoralArm/kI", 0);
	public static LoggedTunableNumber kD = new LoggedTunableNumber("Arms/CoralArm/kD", 0);
	public static LoggedTunableNumber kS = new LoggedTunableNumber("Arms/CoralArm/kS", 0.16);
	public static LoggedTunableNumber kV = new LoggedTunableNumber("Arms/CoralArm/kV", 0.8);
	public static LoggedTunableNumber kA = new LoggedTunableNumber("Arms/CoralArm/kA", 0);
	public static LoggedTunableNumber maxVelocity = new LoggedTunableNumber("Arms/CoralArm/max velocity",
			3);
	public static LoggedTunableNumber maxAcceleration = new LoggedTunableNumber("Arms/CoralArm/max acceleration",
			12);

			

	public static final double momentOfInertia = 0.775;

	public static final boolean simulateGravity = true;
	public static final double stateMarginOfError = 0.225;

	public enum ArmStates {
		CORAL_PICKUP(-0.68),
		CORAL_STATION(Math.PI /2),

		L1(Math.PI/4),
		HAND_OFF(2.27),
		STOW(2.2),
		STOPPED(0),
		CLIMB(1.9);

		public final double setPointRad;

		private ArmStates(double setPoint) {
			this.setPointRad = setPoint;
		}
	}
}