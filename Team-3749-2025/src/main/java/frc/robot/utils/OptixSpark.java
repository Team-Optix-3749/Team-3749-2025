package frc.robot.utils;

import java.util.function.BiFunction;
import java.util.function.Function;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class OptixSpark {

    private SparkBase motor;
    private SparkClosedLoopController controller;
    private ClosedLoopConfig controllerConfig = new ClosedLoopConfig();
    private SparkBaseConfig motorConfig = new SparkMaxConfig();
    private EncoderConfig encoderConfig = new EncoderConfig();
    private Function<Double, Double> wrapping = (input) -> input;
    private BiFunction<Double, Double, Boolean> deadband = (input, setpoint) -> false;

    /**
     * 
     * @param id motor ID
     * 
     */
    public OptixSpark(int id, Type type) {
        if (type == Type.SPARKFLEX) {
            motor = new SparkFlex(id, MotorType.kBrushless);
        } else {
            motor = new SparkMax(id, MotorType.kBrushless);
        }
        controller = motor.getClosedLoopController();

    }

    public double getPosition() {
        double position = motor.getEncoder().getPosition();
        position = wrapping.apply(position);
        return position;
    }

    public double getVelocity() {
        return motor.getEncoder().getVelocity();
    }

    public double getAppliedVolts() {
        return motor.getBusVoltage() * motor.getAppliedOutput();
    }

    public double getBusVolts(){
        return motor.getBusVoltage();
    }

    public double getCurrent() {
        return motor.getOutputCurrent();
    }

    public double getTemperature() {
        return motor.getMotorTemperature();
    }

    public SparkBaseConfig getConfig() {
        return motorConfig;
    }

    public SparkClosedLoopController getController() {
        return controller;
    }

    public SparkAbsoluteEncoder getAbsoluteEncoder() {
        return motor.getAbsoluteEncoder();
    }

    public SparkBase getSpark() {
        return motor;
    }

    public void setPosition(double position) {
        motor.getEncoder().setPosition(position);
    }

    public void setVoltage(double volts) {
        motor.setVoltage(volts);
    }

    public void setVelocityControl(double setpointVelocity, double feedforward) {
        controller.setReference(setpointVelocity, ControlType.kVelocity, ClosedLoopSlot.kSlot0, feedforward);

    }

    public void setVelocityControl(double setpointVelocity, double feedforward, ClosedLoopSlot slot) {
        controller.setReference(setpointVelocity, ControlType.kVelocity, slot, feedforward);

    }

    /**
     * should be called every periodic, even if the positionSetpoint and feedforward
     * values do not change
     * 
     * @param positionSetpoint
     * @param feedforward
     */

    public void setPositionControl(double positionSetpoint, double feedforward) {
        if (deadband.apply(getPosition(), wrapping.apply(positionSetpoint))) {
            controller.setReference(positionSetpoint, ControlType.kPosition, ClosedLoopSlot.kSlot3, feedforward);
        } else {

            controller.setReference(positionSetpoint, ControlType.kPosition, ClosedLoopSlot.kSlot0, feedforward);
        }

    }

    /**
     * should be called every periodic, even if the positionSetpoint and feedforward
     * values do not change
     * 
     * @param positionSetpoint
     * @param feedforward
     * @param slot
     */

    public void setPositionControl(double positionSetpoint, double feedforward, ClosedLoopSlot slot) {
        if (deadband.apply(getPosition(), wrapping.apply(positionSetpoint))) {
            controller.setReference(positionSetpoint, ControlType.kPosition, slot, feedforward);
        } else {

            controller.setReference(positionSetpoint, ControlType.kPosition, slot, feedforward);
        }

    }

    /**
     * Within this margin of 0, motor will run with PID constants of 0
     * 
     * @param deadband
     */
    public void setPositionDeadband(double deadband) {
        this.deadband = (input, setpoint) -> {
            if (input < setpoint + deadband) {
                return true;
            }
            return false;
        };

    }

    /**
     * default unit is motor rotations
     * 
     * @param factor
     */
    public void setPositionConversionFactor(double factor) {
        encoderConfig.positionConversionFactor(factor);

        motorConfig.apply(encoderConfig);

    }

    /**
     * default unit is motor rotations per minute
     * 
     * @param factor
     */
    public void setVelocityConversionFactor(double factor) {
        encoderConfig.velocityConversionFactor(factor);

        motorConfig.apply(encoderConfig);

    }

    /**
     * 
     * @param stallLimit
     * @param freeLimit
     */
    public void setCurrentLimit(int stallLimit, int freeLimit) {
        motorConfig.smartCurrentLimit(stallLimit, freeLimit);

    }

    /**
     * 
     * @param p
     * @param i
     * @param d
     * @param f
     * @param slot Note: slot 3 should always have values of 0, wich is also the
     *             default
     */
    public void setPID(double p, double i, double d, ClosedLoopSlot slot) {
        controllerConfig.pid(p, i, d, slot);
        motorConfig.apply(controllerConfig);

    }

    /**
     * If min = max, then wrapping will be disabled
     * 
     * @param min
     * @param max
     */
    public void setPositionWrapping(double min, double max) {
        if (min == max) {
            controllerConfig.positionWrappingEnabled(false);
            wrapping = (input) -> input;
            return;
        }
        wrapping = (input) -> {
            while (input < min) {
                input += max - min;
            }
            while (input > max) {
                input -= max - min;
            }
            return input;

        };

        controllerConfig.positionWrappingEnabled(true);

        controllerConfig.positionWrappingInputRange(min, max);
        motorConfig.apply(controllerConfig);

    }

    public void setInverted(boolean inverted) {
        motorConfig.inverted(inverted);

    }

    public void setAbsoluteEncoderInverted(boolean inverted){
        motorConfig.absoluteEncoder.inverted(inverted);

    }

    public void setBrakeMode(boolean isBrakeMode) {
        if (isBrakeMode) {
            motorConfig.idleMode(IdleMode.kBrake);
        } else {
            motorConfig.idleMode(IdleMode.kCoast);
        }

    }

    public void setFollow(OptixSpark spark) {
        motorConfig.follow(spark.getSpark());
    }

    public void setControlEncoder(FeedbackSensor sensor){
        controllerConfig.feedbackSensor(sensor);
        motorConfig.apply(controllerConfig);
    }

    /**
     * Configure motor with internal settings applied
     */
    public void applyConfig() {
        motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    }

    /**
     * Configure motor with external settings applied
     * 
     * @param config
     */
    public void applyConfig(SparkBaseConfig config) {
        motorConfig = config;
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public static enum Type {
        SPARKMAX,
        SPARKFLEX
    }
}