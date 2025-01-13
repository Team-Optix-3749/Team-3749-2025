package frc.robot.subsystems.rollers;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

public class RollerConstants {
    public enum RollerStates {
        RUN,
        MAINTAIN,
        STOP
    }

    public static class CoralConstants {
        public static PIDController coralPIDController = new PIDController(1, 0, 0);
        public static SimpleMotorFeedforward coralFeedForward = new SimpleMotorFeedforward(0, 0, 0);
    }

    public static class AlgaeConstants {
        public static PIDController algaePIDController = new PIDController(1, 0, 0);
        public static SimpleMotorFeedforward algaeFeedForward = new SimpleMotorFeedforward(0, 0, 0);
    }

    public static class ScoringConstants {
        public static PIDController scoringConstants = new PIDController(1, 0, 0);
        public static SimpleMotorFeedforward scoringFeedForward = new SimpleMotorFeedforward(0, 0, 0);
    }
}
