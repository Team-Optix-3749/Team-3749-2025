package frc.robot.subsystems.arm.climb.real;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.arm.climb.ClimbArmIO;
import frc.robot.subsystems.arm.climb.ClimbArmIO.ArmData;
import frc.robot.utils.MiscConstants.SimConstants;

/**
 * IO implementation for an arm subsystem's sparkmax motors, using two SparkMax motors
 * and an absolute encoder to track the arm's position.
 *
 * @author Elise Lin
 */
public class ClimbArmSparkMax implements ClimbArmIO {

    // Define the front and back SparkMax motor controllers
    private SparkMax frontMotor;
    private SparkMax backMotor;
    
    // Configuration settings for the front and back motors
    private SparkMaxConfig frontMotorConfig = new SparkMaxConfig();
    private SparkMaxConfig backMotorConfig = new SparkMaxConfig();

    // Absolute encoder used to track the position of the arm
    private SparkAbsoluteEncoder absoluteEncoder;
    
    // Variable to store the absolute position value from the encoder
    private double absolutePos;

    // Voltage input to the motors and velocity tracking for the arm
    private double inputVolts = 0;
    private double previousVelocity = 0;
    private double velocity = 0;

    /**
     * Constructor that creates an instance of the ClimbArmSparkMax using two motors.
     * Configures the motors with appropriate settings and initializes the encoder.
     * 
     * @param frontMotorId ID of the front motor
     * @param backMotorId ID of the back motor
     */
    public ClimbArmSparkMax(int frontMotorId, int backMotorId) {
        // Initialize the motors with the given IDs, assuming brushless motors
        frontMotor = new SparkMax(frontMotorId, MotorType.kBrushless);
        backMotor = new SparkMax(backMotorId, MotorType.kBrushless);

        // Configure the front motor with specific settings (current limits, encoder settings, etc.)
        frontMotorConfig.smartCurrentLimit(ArmConstants.NEOStallLimit, ArmConstants.NEOFreeLimit);
        frontMotorConfig.encoder.inverted(false);  // Front motor encoder is not inverted
        frontMotorConfig.inverted(false);  // Front motor is not inverted
        frontMotorConfig.idleMode(IdleMode.kBrake);  // Set motor to brake when idle
        frontMotorConfig.encoder.positionConversionFactor(2 * Math.PI);  // Conversion factor for position (radians)
        frontMotorConfig.encoder.velocityConversionFactor(2 * Math.PI / 60.0);  // Conversion factor for velocity (radians per second)

        // Apply the front motor configuration
        frontMotor.configure(frontMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Configure the back motor with similar settings but with some differences (inversion for back motor)
        backMotorConfig.apply(frontMotorConfig);  // Start with the same config as front motor
        backMotorConfig.encoder.inverted(true);  // Back motor encoder is inverted
        backMotorConfig.inverted(true);  // Back motor is inverted

        // Apply the back motor configuration
        backMotor.configure(backMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Get the absolute encoder from the front motor and set its position
        absoluteEncoder = frontMotor.getAbsoluteEncoder();
        absolutePos = absoluteEncoder.getPosition();

        // Set the initial position of both motors' encoders to match the absolute encoder's position
        frontMotor.getEncoder().setPosition(absolutePos);
        backMotor.getEncoder().setPosition(absolutePos);
    }

    /**
     * Sets the idle mode (brake or coast) for both motors.
     * 
     * @param idleMode Desired idle mode for the motors
     */
    public void setIdleMode(IdleMode idleMode) {
        // Set the idle mode for both motors and apply the configurations
        frontMotorConfig.idleMode(idleMode);
        backMotorConfig.idleMode(idleMode);
        frontMotor.configure(frontMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        backMotor.configure(backMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    /**
     * Updates the motor data with the current position, velocity, acceleration, current, voltage, and temperature.
     * It averages the data from both front and back motors for position and velocity.
     * 
     * @param data The ArmData object to be updated with new values
     */
    @Override
    public void updateData(ArmData data) {
        // Store previous velocity for acceleration calculation
        previousVelocity = velocity;
        
        // Calculate the average velocity from both motors
        velocity = (frontMotor.getEncoder().getVelocity() + backMotor.getEncoder().getVelocity()) / 2;
        
        // Calculate the average position and assign it to the ArmData object
        data.positionUnits = (frontMotor.getEncoder().getPosition() + backMotor.getEncoder().getVelocity()) / 2;
        
        // Assign the current velocity to the ArmData object
        data.velocityUnits = velocity;
        
        // Calculate the arm's acceleration by comparing the current and previous velocities
        data.accelerationUnits = (velocity - previousVelocity) / SimConstants.loopPeriodSec;
        
        // Get the current drawn by both motors and assign to the ArmData object
        data.frontMotorCurrentAmps = frontMotor.getOutputCurrent();
        data.backMotorCurrentAmps = backMotor.getOutputCurrent();
        
        // Assign the input voltage to the ArmData object
        data.inputVolts = inputVolts;
        
        // Calculate the applied voltage on each motor and assign to the ArmData object
        data.frontMotorAppliedVolts = frontMotor.getBusVoltage() * frontMotor.getAppliedOutput();
        data.backMotorAppliedVolts = backMotor.getBusVoltage() * backMotor.getAppliedOutput();
        
        // Get and assign the motor temperatures to the ArmData object
        data.frontMotorTempCelcius = frontMotor.getMotorTemperature();
        data.backMotorTempCelcius = backMotor.getMotorTemperature();
    }

    /**
     * Set the voltage to be applied to both the front and back motors.
     * The voltage is clamped to a range of -12 to 12 volts 
     * 
     * @param volts The voltage to be applied to the motors
     */
    @Override
    public void setVoltage(double volts) {
        // Apply deadband to remove small input voltages (within ±0.05V range)
        inputVolts = MathUtil.applyDeadband(inputVolts, 0.05);
        
        // Clamp the input voltage to be within the range of -12 to 12 volts
        inputVolts = MathUtil.clamp(volts, -12, 12);
        
        // Set the voltage on both motors
        frontMotor.setVoltage(inputVolts);
        backMotor.setVoltage(inputVolts);
    }
}
