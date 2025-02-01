package frc.robot.subsystems.arm;

/**
 * Interface file for arm subsystem
 *
 * @author Weston Gardner
 */
public interface ClimbArmIO {

	public static class ArmData {
		public double positionUnits = 0;
		public double velocityUnits = 0;
		public double accelerationUnits = 0;
		public double inputVolts = 0;
		public double firstMotorCurrentAmps = 0;
		public double secondMotorCurrentAmps = 0;
		public double firstMotorAppliedVolts = 0;
		public double secondMotorAppliedVolts = 0;
		public double firstMotorTempCelcius = 0;
		public double secondMotorTempCelcius = 0;
	}

	/* Updates the set of loggable inputs. */
	public default void updateData(ArmData data) {
	};

	/* Run the motor at the specified voltage. */
	public default void setVoltage(double volts) {
	};

	/* Enable or disable brake mode on the motor. */
	public default void setBrakeMode(boolean enable) {
	};
}
