package frc.robot.subsystems.rollers.coral;

public interface CoralIO {

    public static class CoralData
    {
        public double intakeVoltage;
        public double goalVoltage; // if this is unnecessary because intakeVoltage == goalVoltage then delete this
        public double intakeTempCelsius;
        public double intakeVelocityRadPerSec;
        public boolean hasCoral; //for beam break most likely 
    }

    public default void setVoltage(double volts)
    {

    }

    public default void updateData(CoralData data)
    {

    }
    
}
