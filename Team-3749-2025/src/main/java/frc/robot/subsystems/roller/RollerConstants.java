package frc.robot.subsystems.roller;
public class RollerConstants {
    public static final class Algae {
        public static final int motorId = 1;
        public static final double momentOfInertia = 0.04;
        public static final double gearRatio = 1.0;
        public static final double measurementNoise = 0.0;

        public static final double kPSim = 10.0;
        public static final double kISim = 0.0;
        public static final double kDSim = 0.0;
        public static final double kSSim = 0.0;
        public static final double kVSim = 0.02;
        public static final double kASim = 0.0;

        public static final double velocity = 5.0;
    }
    public static final class Coral {
        public static final int motorId = 2; 
        public static final double momentOfInertia = 0.04;
        public static final double gearRatio = 1.0;
        public static final double measurementNoise = 0.0;

        public static final double kPSim = 10.0;
        public static final double kISim = 0.0;
        public static final double kDSim = 0.0;
        public static final double kSSim = 0.0;
        public static final double kVSim = 0.02;
        public static final double kASim = 0.0;

        public static final double velocity = 7.0;
    }
    public static final class Scoring {
        public static final int motorId = 3; 
        public static final double momentOfInertia = 0.04;
        public static final double gearRatio = 1.0;
        public static final double measurementNoise = 0.0;

        public static final double kPSim = 10.0;
        public static final double kISim = 0.0;
        public static final double kDSim = 0.0;
        public static final double kSSim = 0.0;
        public static final double kVSim = 0.02;
        public static final double kASim = 0.0;

        public static final double velocity = 10.0;
    }
    
    public enum RollerStates {
        RUN,
        MAINTAIN,
        STOP
    }

    public enum Implementations {
        CORAL,
        ALGAE,
        SCORING
    }
}
