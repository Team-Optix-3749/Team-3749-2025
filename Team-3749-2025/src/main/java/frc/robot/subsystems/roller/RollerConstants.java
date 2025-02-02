package frc.robot.subsystems.roller;
public class RollerConstants {
    public static final class Algae {
        public static final int motorId = 1;
        public static final double momentOfInertia = 0.04;
        public static final double gearRatio = 1.0;
        public static final double measurementNoise = 0.0;

        public static final double kPVelocity = 10.0;
        public static final double kIVelocity = 0.0;
        public static final double kDVelocity = 0.0;
        public static final double kSVelocity = 0.0;
        public static final double kVVelocity = 0.02;
        public static final double kAVelocity = 0.0;

        public static final double kPPosition = 15.0;
        public static final double kIPosition = 0.0;
        public static final double kDPosition = 10.0;

        public static final double velocity = 5.0;
    }
    public static final class Coral {
        public static final int motorId = 2; 
        public static final double momentOfInertia = 0.04;
        public static final double gearRatio = 1.0;
        public static final double measurementNoise = 0.0;

        public static final double kPVelocity = 10.0;
        public static final double kIVelocity = 0.0;
        public static final double kDVelocity = 0.0;
        public static final double kSVelocity = 0.0;
        public static final double kVVelocity = 0.02;
        public static final double kAVelocity = 0.0;

        public static final double kPPosition = 15.0;
        public static final double kIPosition = 0.0;
        public static final double kDPosition = 10.0;

        public static final double velocity = 7.0;
        public static final double scoreVelocity = -7.0;
    }
    public static final class Scoring {
        public static final int motorId = 3; 
        public static final double momentOfInertia = 0.04;
        public static final double gearRatio = 1.0;
        public static final double measurementNoise = 0.0;

        public static final double kPVelocity = 10.0;
        public static final double kIVelocity = 0.0;
        public static final double kDVelocity = 0.0;
        public static final double kSVelocity = 0.0;
        public static final double kVVelocity = 0.02;
        public static final double kAVelocity = 0.0;

        public static final double kPPosition = 15.0;
        public static final double kIPosition = 0.0;
        public static final double kDPosition = 10.0;

        public static final double velocity = 10.0;
        public static final double scoreVelocity = -10.0;
    }
    
    public enum RollerStates {
        INTAKE,
        MAINTAIN,
        STOP,
        SCORE
    }

    public enum Implementations {
        ALGAE,
        CORAL,
        SCORING
    }
}
