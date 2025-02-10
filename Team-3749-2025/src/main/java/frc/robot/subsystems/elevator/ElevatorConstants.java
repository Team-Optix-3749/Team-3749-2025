package frc.robot.subsystems.elevator;

/**
 * Elevator constants
 * 
 * @author Dhyan Soni
 * @author Andrew Ge
 */

import edu.wpi.first.math.util.Units;

public class ElevatorConstants {
    public static class ElevatorSpecs {
        public static final double gearing = 6.0;
        public static final double carriageMassKg = 12;
        public static final double drumRadiusMeters = Units.inchesToMeters(2);
        public static final double minHeightMeters = Units.inchesToMeters(0);
        public static final double maxHeightMeters = Units.feetToMeters(6); // remeasure maxV and A
        // public static final boolean simulateGravity = true;
        public static final double startingHeightMeters = 0;

        public static final double baseHeight = Units.feetToMeters(3.25);

        public static int[] motorIds = { 19, 20 };
        public static boolean[] motorInverted = { false, true };



        public static int zeroOffset = 0;
    }

    public static class ElevatorControl {
        public static final double kP = 0.02;
        public static final double kI = 0.0;
        public static final double kD = 0;
        public static final double kS = 0;
        public static final double kG = 2.2977;
        public static final double kV = 2.35; // 12 - 2.3 / 4.139
        public static final double kA = 0;
        public static final double maxV = 4.139;
        public static final double maxA = 3.988; // change in velocity / seconds
    }

    public enum ElevatorStates {
        STOP,
        L1,
        L2,
        L3,
        L4,
        MAX,
        STOW
    }

    public static class StateHeights {
        public static final double l1Height = Units.inchesToMeters(18);
        public static final double l2Height = Units.inchesToMeters(31.875 - 28.5 + 14.428 - 4.9 + 0.6 + 3);
        public static final double l3Height = Units.inchesToMeters(47.625 - 28.5 + 14.428 - 4.9 + 0.6 + 3);
        public static final double l4Height = Units.inchesToMeters(72 - 28.5 + 14.428 - 4.9 + 0.6 + 3);
    }
}
