package frc.robot.subsystems.roller;

/**
 * Constants for roller subsystem
 * 
 * @author Lilian Wu
 */

import frc.robot.utils.LoggedTunableNumber;

public class RollerConstants {

        public static final class Coral {
                public static final int motorId = 15;
                public static final boolean inverted = true;

                public static final double momentOfInertia = 0.04;
                public static final double gearRatio = 3;
                public static final double measurementNoise = 0.0;

                public static LoggedTunableNumber kPVelocity = new LoggedTunableNumber(
                                " rollers/coralRoller/kPVelocity", 0.0);
                public static LoggedTunableNumber kIVelocity = new LoggedTunableNumber(
                                " rollers/coralRoller/kIVelocity", 0.0);
                public static LoggedTunableNumber kDVelocity = new LoggedTunableNumber(
                                " rollers/coralRoller/kDVelocity", 0.0);
                public static LoggedTunableNumber kSVelocity = new LoggedTunableNumber(
                                " rollers/coralRoller/kSVelocity", 0.205);
                public static LoggedTunableNumber kVVelocity = new LoggedTunableNumber(
                                " rollers/coralRoller/kVVelocity", 0.0594);
                public static LoggedTunableNumber kAVelocity = new LoggedTunableNumber(
                                " rollers/coralRoller/kAVelocity", 0.04);

                public static LoggedTunableNumber kPPosition = new LoggedTunableNumber(
                                " rollers/coralRoller/kPPosition", 0.5);
                public static LoggedTunableNumber kIPosition = new LoggedTunableNumber(
                                " rollers/coralRoller/kIPosition", 0.0);
                public static LoggedTunableNumber kDPosition = new LoggedTunableNumber(
                                " rollers/coralRoller/kDPosition", 0.0);

                public static LoggedTunableNumber maxVelocity = new LoggedTunableNumber(
                                " rollers/coralRoller/maxVelocity", 194.0);
                public static LoggedTunableNumber maxAcceleration = new LoggedTunableNumber(
                                " rollers/coralRoller/maxAcceleration", 415.0);

                public static LoggedTunableNumber intakeVelocity = new LoggedTunableNumber(
                                " rollers/coralRoller/intakeVelocitySetpoint", 150.0);
                public static LoggedTunableNumber outtakeVelocity = new LoggedTunableNumber(
                                " rollers/coralRoller/outtakeVelocitySetpoint", -40.0);

        }

        public static final class Scoring {
                public static final int motorId = 21;
                public static final boolean inverted = true;

                public static final double momentOfInertia = 0.04;
                public static final double gearRatio = 4;
                public static final double measurementNoise = 0.0;

                public static LoggedTunableNumber kPVelocity = new LoggedTunableNumber(
                                " rollers/scoringRoller/kPVelocity", 0.0);
                public static LoggedTunableNumber kIVelocity = new LoggedTunableNumber(
                                " rollers/scoringRoller/kIVelocity", 0.0);
                public static LoggedTunableNumber kDVelocity = new LoggedTunableNumber(
                                " rollers/scoringRoller/kDVelocity", 0.0);
                public static LoggedTunableNumber kSVelocity = new LoggedTunableNumber(
                                " rollers/scoringRoller/kSVelocity", 0.14);
                public static LoggedTunableNumber kVVelocity = new LoggedTunableNumber(
                                " rollers/scoringRoller/kVVelocity", 0.0619);
                public static LoggedTunableNumber kAVelocity = new LoggedTunableNumber(
                                " rollers/scoringRoller/kAVelocity", 0.0142);

                public static LoggedTunableNumber kPPosition = new LoggedTunableNumber(
                                " rollers/scoringRoller/kPPosition", .25);
                public static LoggedTunableNumber kIPosition = new LoggedTunableNumber(
                                " rollers/scoringRoller/kIPosition", 0.0);
                public static LoggedTunableNumber kDPosition = new LoggedTunableNumber(
                                " rollers/scoringRoller/kDPosition", 0.0);

                public static LoggedTunableNumber maxVelocity = new LoggedTunableNumber(
                                " rollers/scoringRoller/maxVelocity", 194);
                public static LoggedTunableNumber maxAcceleration = new LoggedTunableNumber(
                                " rollers/scoringRoller/maxAcceleration", 412);

                public static final int sensorPort = 7;

                public static final double reverseDistance = 7;
        }

        public enum RollerStates {
                INTAKE(-200, 85, 200),
                MAINTAIN(0, 0, 0),
                OUTTAKE(-50, -42.5, Scoring.maxVelocity.get()), // -42.5
                SCORE(0, -50, Scoring.maxVelocity.get()), // -42.5
                STOP(0, 0, 0);

                public final double algaeVelocity;
                public final double coralVelocity;
                public final double scoringVelocity;

                private RollerStates(double algae, double coral, double scoring) {
                        algaeVelocity = algae;
                        coralVelocity = coral;
                        scoringVelocity = scoring;
                }

        }

        public enum Implementations {

                CORAL,
                SCORING
        }
}
