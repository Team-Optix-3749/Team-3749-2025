package frc.robot.subsystems.roller;

public class RollerConstants {
    public static final int algaeMotorId = 1;
    public static final int coralMotorId = 2; 
    public static final int scoringMotorId = 3; 

    public static final double momentOfInertia = 0.04;
    public static final double gearRatio = 1.0;
    public static final double measurementNoise = 0.0;

    // PID and FF values
    public static final double kPSim = 10.0;
    public static final double kISim = 0.0;
    public static final double kDSim = 0.0;
    public static final double kSSim = 0.0;
    public static final double kVSim = 0.02;
    public static final double kASim = 0.0;

    public static final double coralVelocity = 7.0;
    public static final double algaeVelocity = 5.0;
    public static final double scoringVelocity = 10.0;
    public static final double holdVoltage = 2.0;

    public enum RollerStates {
        RUN,
        MAINTAIN,
        STOP
    }
}
