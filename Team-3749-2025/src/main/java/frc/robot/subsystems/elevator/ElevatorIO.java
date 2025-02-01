package frc.robot.subsystems.elevator;

/**
 * IO implementation for elevator subsystem
 * 
 * @author Dhyan Soni
 */

public interface ElevatorIO {
    public static class ElevatorData {
        public double positionMeters = 0;
        public double velocityMetersPerSecond = 0;
        public double accelerationMetersPerSecondSquared = 0;
        public double leftCurrentAmps = 0;
        public double rightCurrentAmps = 0;
        public double inputVolts = 0;
        public double leftAppliedVolts = 0;
        public double rightAppliedVolts = 0;
        public double leftTempCelcius = 0;
        public double rightTempCelcius = 0;
    }

    /**Updates the loggable inputs */
    public default void updateData(ElevatorData data) {
    };

    /**Runs the motor as a set voltage */
    public default void setVoltage(double volts) {
    };
}
