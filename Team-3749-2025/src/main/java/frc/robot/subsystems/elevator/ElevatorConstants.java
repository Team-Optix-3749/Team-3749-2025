package frc.robot.subsystems.elevator;

import frc.robot.utils.LoggedTunableNumber;

/**
 * Elevator constants
 * 
 * @author Dhyan Soni
 * @author Andrew Ge
 */

import edu.wpi.first.math.util.Units;

public class ElevatorConstants {
    public static class ElevatorSpecs {
        public static final double gearing = 16 * (24.0 / 22.0);
        public static final double carriageMassKg = Units.lbsToKilograms(54);
        // 22 TOOTH, 1/4 in pitch, divide by 2pi to go from circumfrence to radius
        public static final double drumRadiusMeters = (Units.inchesToMeters(22.0 / 4.0) / (2 * Math.PI));
        public static final double minHeightMeters = 0;
        public static final double maxHeightMeters = Units.feetToMeters(6); // remeasure maxV and A
        // public static final boolean simulateGravity = true;
        public static final double startingHeightMeters = 0;

        public static final double baseHeight = Units.feetToMeters(3.25);

        public static int[] motorIds = { 19, 20 };
        public static boolean[] motorInverted = { false, true };

        public static int zeroOffset = 0;
    }

    public static final double stateMarginOfError = 0.1;

    public static class ElevatorControl {
        public static LoggedTunableNumber kG = new LoggedTunableNumber("Elevator/kG", 0.32);
        public static LoggedTunableNumber kP = new LoggedTunableNumber("Elevator/kP", 12);
        public static LoggedTunableNumber kI = new LoggedTunableNumber("Elevator/kI", 0);
        public static LoggedTunableNumber kD = new LoggedTunableNumber("Elevator/kD", 0);
        public static LoggedTunableNumber kS = new LoggedTunableNumber("Elevator/kS", 0.16);
        public static LoggedTunableNumber kV = new LoggedTunableNumber("Elevator/kV", 7.77);
        public static LoggedTunableNumber kA = new LoggedTunableNumber("Elevator/kA", 0.27); // 1.72
        public static LoggedTunableNumber maxVelocity = new LoggedTunableNumber("Elevator/max velocity",
                1.415);
        public static LoggedTunableNumber maxAcceleration = new LoggedTunableNumber(
                "Elevator/max acceleration",
                4.1);
    }

    public enum ElevatorStates {
        STOP(Units.inchesToMeters(0)),
        L1(Units.inchesToMeters(12)),
        L2(Units.inchesToMeters(15.35)),
        L3(.742),
        L4(1.3),
        SOURCE(Units.inchesToMeters(30)),
        ALGAE_LOW(Units.inchesToMeters(.4)),
        ALGAE_HIGH(Units.inchesToMeters(16)),
        MAX(Units.feetToMeters(6)),
        STOW(Units.inchesToMeters(.75));
        // STOP(Units.inchesToMeters(0)),
        // L1(Units.inchesToMeters(0)),
        // L2(Units.inchesToMeters(0)),
        // L3(Units.inchesToMeters(0)),
        // L4(0),
        // SOURCE(Units.inchesToMeters(0)),
        // ALGAE_LOW(Units.inchesToMeters(0)),
        // ALGAE_HIGH(Units.inchesToMeters(0)),
        // MAX(Units.feetToMeters(0)),
        // STOW(Units.inchesToMeters(0));

        public double heightMeters;

        private ElevatorStates(double heightMeters) {
            this.heightMeters = heightMeters;
        }
    }
}