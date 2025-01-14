package frc.robot.subsystems.rollers.algae.intake;

public interface AlgaeIntakeIO {

    public static class AlgaeIntakeData
    {
        public double busVoltage;
        public double goalVoltage; // if this is unnecessary because intakeVoltage == goalVoltage then delete this
        public double tempCelsius;
        public double velocityRadPerSec;
        public boolean hasAlgae; //for beam break most likely
    }


    public default void stop()
    {
        
    }

    public default void setVoltage(double volts)
    {
      
    }

    public default void updateData(AlgaeIntakeData data)
    {

    }
    
}
