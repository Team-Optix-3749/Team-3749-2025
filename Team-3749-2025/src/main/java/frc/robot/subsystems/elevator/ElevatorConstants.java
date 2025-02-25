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
        public static boolean[] motorInverted = { true, false };

        public static int zeroOffset = 0;
    }

    public static final double stateMarginOfError = 0.1;

    public static class ElevatorControl {
        public static LoggedTunableNumber kG = new LoggedTunableNumber("/subsystems/elevator/kG", 0.32);
        public static LoggedTunableNumber kP = new LoggedTunableNumber("/subsystems/elevator/kP", 12);
        public static LoggedTunableNumber kI = new LoggedTunableNumber("/subsystems/elevator/kI", 0);
        public static LoggedTunableNumber kD = new LoggedTunableNumber("/subsystems/elevator/kD", 0);
        public static LoggedTunableNumber kS = new LoggedTunableNumber("/subsystems/elevator/kS", 0.16
        );
        public static LoggedTunableNumber kV = new LoggedTunableNumber("/subsystems/elevator/kV", 7.77);
        public static LoggedTunableNumber kA = new LoggedTunableNumber("/subsystems/elevator/kA", 0.27
        ); // 1.72
        public static LoggedTunableNumber maxVelocity = new LoggedTunableNumber("/subsystems/elevator/max velocity",
                1.415);
        public static LoggedTunableNumber maxAcceleration = new LoggedTunableNumber(
                "/subsystems/elevator/max acceleration",
                4.1);
    }

    public enum ElevatorStates {
        STOP(Units.inchesToMeters(2)),
        L1(Units.inchesToMeters(12)),
        L2(Units.inchesToMeters(15.35)),
        L3(Units.inchesToMeters(31.25)),
        L4(Units.inchesToMeters(54.65)),
        SOURCE(Units.inchesToMeters(30)),
        ALGAE_LOW(Units.inchesToMeters(31.875)),
        ALGAE_HIGH(Units.inchesToMeters(47.625)),
        MAX(Units.feetToMeters(6)),
        STOW(Units.inchesToMeters(2));

        public double heightMeters;

        private ElevatorStates(double heightMeters)
        {
            this.heightMeters = heightMeters;
        }
    }
}