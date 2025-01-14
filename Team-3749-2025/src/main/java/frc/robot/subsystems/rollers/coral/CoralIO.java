package frc.robot.subsystems.rollers.coral;

public interface CoralIO {

    public static class CoralData
    {
        public double busVoltage;
        public double goalVoltage; // if this is unnecessary because intakeVoltage == goalVoltage then delete this
        public double tempCelsius;
        public double velocityRadPerSec;
        public boolean hasCoral; //for beam break most likely
    }

    public default void stop()
    {
        
    }

    public default void setVoltage(double volts)
    {
      
    }

    public default void updateData(CoralData data)
    {

    }
    
}
