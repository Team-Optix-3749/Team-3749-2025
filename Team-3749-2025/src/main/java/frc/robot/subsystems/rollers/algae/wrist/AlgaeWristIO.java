package frc.robot.subsystems.rollers.algae.intake;

public interface AlgaeWristIO {

    public static class AlgaeIntakeData
    {
        public double busVoltage;
        public double goalVoltage; // if this is unnecessary because intakeVoltage == goalVoltage then delete this
        public double tempCelsius;
        public double velocityRadPerSec;
        public boolean hasAlgae; //for beam break most likely
    }

    public static class AlgaeWristData
    {
        public double busVoltage;
        public double goalVoltage; // if this is unnecessary because intakeVoltage == goalVoltage then delete this
        public double tempCelsius;
        public double velocityRadPerSec;
        public double currentArmAngle;
    }


    public default void stop()
    {
        
    }

    public default void setVoltage(double volts)
    {
      
    }

    public default void updateData(AlgaeWristData data)
    {

    }

    public default void updateData(AlgaeIntakeData data)
    {

    }
    
}
