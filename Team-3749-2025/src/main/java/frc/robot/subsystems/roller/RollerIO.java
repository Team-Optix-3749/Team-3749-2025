package frc.robot.subsystems.roller;
/**
 * IO for roller subsystem
 * 
 * @author Lilian Wu
 */
public interface RollerIO {
    public static class RollerData {
        public double rollerAppliedVolts = 0.0;
        public double rollerVelocityRadPerSec = 0.0;
        public double rollerTempCelcius = 0.0;
        public double currentAmps = 0.0;
        public double rollerPositionRad = 0.0;
        public boolean sensorTripped = false;
    }

    public default void updateData(RollerData data) {

    }

    public default void setVoltage(double rollerVolts) {

    }

    public default void setBrakeMode(boolean enable) {

    }
    
    public default void setVelocity(double setpointVelocity, double feedforward) {
    }

    
    public default void setPosition(double setpointVelocity, double feedforward) {
    }

}
