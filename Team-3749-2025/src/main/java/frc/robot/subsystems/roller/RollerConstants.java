package frc.robot.subsystems.roller;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.Robot;

public class RollerConstants {

    public static final int algaeMotorId = 2; // example
    public static final int coralMotorId = 1; // example
    public static final int scoringMotorId = 2; // example

    public static final double algaeGearRatio = 1; // example
    public static final double coralGearRatio = 1; // example
    public static final double scoringGearRatio = 1; // example

    // example PID Sim Values
    public static double kSAlgaeSim = 0.2;
    public static double kVAlgaeSim = 1.98;
    public static double kAAlgaeSim = 0.0;

    public static double kSCoralSim = 2.0;
    public static double kVCoralSim = 0.4;
    public static double kACoralSim = 0.0;

    public static double kSScoringSim = 0.1;
    public static double kVScoringSim = 2.1;
    public static double kAScoringSim = 0.0;

    
}
