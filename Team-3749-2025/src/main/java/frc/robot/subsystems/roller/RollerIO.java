package frc.robot.subsystems.roller;

public interface RollerIO {
    public static class RollerData {
        public double rollerAppliedVolts = 0.0;
        public double rollerVelocityRadPerSec = 0.0;
        public double rollerTempCelcius = 0.0;
        public double currentAmps = 0.0;
        public double rollerPositionRotations = 0.0;
        public boolean sensorTripped = false;

    }

    public default void updateData(RollerData data) {

    }

    public default void setVoltage(double rollerVolts) {

    }

    public default void setBrakeMode(boolean enable) {

    }

}
