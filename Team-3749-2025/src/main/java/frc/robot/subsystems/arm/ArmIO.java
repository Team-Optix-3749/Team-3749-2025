package frc.robot.subsystems.arm;

/**
 * Interface file for arm subsystem
 * 
 * @author Weston Gardner
 */

public interface ArmIO {

  public static class ArmData {
    public double positionUnits = 0;
    public double velocityUnits = 0;
    public double accelerationUnits = 0;
    public double inputVolts = 0;
    public double appliedVolts = 0;
    public double currentAmps = 0;
    public double tempCelcius = 0;

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
