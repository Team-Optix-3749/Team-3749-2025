package frc.robot.subsystems.arm.coral;

import frc.robot.Robot;
import frc.robot.utils.SysIdTuner;
import frc.robot.utils.UtilityFunctions;
import frc.robot.utils.SysIdTuner.Type;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.arm.coral.CoralArmIO.ArmData;
import frc.robot.utils.MotorData;
import static edu.wpi.first.units.Units.*;

import java.util.Map;
import frc.robot.subsystems.arm.coral.real.CoralArmSparkMax;
import frc.robot.subsystems.arm.coral.sim.CoralArmSim;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

/**
 * Subsystem class for the coral arm
 *
 * @author Weston Gardner
 */
public class CoralArm extends SubsystemBase {

    private CoralArmIO armIO;
    private ArmData data = new ArmData();
    private CoralArmConstants.ArmStates state = CoralArmConstants.ArmStates.STOWED;

    private SysIdTuner sysIdTuner;

    Map<String, MotorData> motorData = Map.of(
            "arm_motor", new MotorData(
                    data.appliedVolts,
                    data.positionRad,
                    data.velocityUnits,
                    data.accelerationUnits));

    SysIdRoutine.Config config = new SysIdRoutine.Config(
            Volts.per(Seconds).of(1), // Voltage ramp rate
            Volts.of(8), // Max voltage
            Seconds.of(8) // Test duration
    );

    // Profiled PID Controller used only for the motion profile, PID within
    // implementation classes
    private ProfiledPIDController profile;
    private ArmFeedforward feedforward;
    private LoggedMechanism2d mechanism2d = new LoggedMechanism2d(3, 3);
    private LoggedMechanismRoot2d armRoot = mechanism2d.getRoot("ArmRoot", 1.8, .4);
    private LoggedMechanismLigament2d armLigament = armRoot
            .append(new LoggedMechanismLigament2d("Coral Arm", CoralArmConstants.armLength_meters, 0));

    private StructPublisher<Pose3d> publisher = NetworkTableInstance.getDefault()
            .getStructTopic("CoralArm Pose", Pose3d.struct).publish();

    /**
     * Constructor for the CoralArm subsystem. Determines if simulation or real
     * hardware is used.
     */
    public CoralArm() {

        // If the robot is in simulation, use the simulated I/O for the arm.
        if (Robot.isSimulation()) {
            armIO = new CoralArmSim();

        } else {
            // If running on real hardware, use SparkMax motors for the arm.
            armIO = new CoralArmSparkMax();
        }
        profile = new ProfiledPIDController(CoralArmConstants.kP.get(), CoralArmConstants.kI.get(),
                CoralArmConstants.kD.get(),
                new TrapezoidProfile.Constraints(CoralArmConstants.maxVelocity.get(),
                        CoralArmConstants.maxAcceleration.get()));
        feedforward = new ArmFeedforward(CoralArmConstants.kS.get(), CoralArmConstants.kG.get(),
                CoralArmConstants.kV.get(),
                CoralArmConstants.kA.get());

        sysIdTuner = new SysIdTuner("coral arm", config, this, armIO::setVoltage, motorData, Type.ROTATIONAL);
        setState(state);
    }

    // GET FUNCTIONS

    /**
     * @return The current arm state (e.g., STOPPED, STOWED, etc.)
     */
    public CoralArmConstants.ArmStates getState() {
        return state;
    }

    /**
     * @return The current position of the arm in radians.
     */
    public double getPositionRad() {
        return data.positionRad; // Return the arm's current position.
    }

    /**
     * @return Whether the arm is in a stable state. Checks if the arm is within a
     *         margin
     *         of error for its set positions.
     */
    public boolean getIsStableState() {

        switch (state) {
            case STOWED:
                return UtilityFunctions.withinMargin(CoralArmConstants.stateMarginOfError,
                        CoralArmConstants.stowSetPoint_rad, data.positionRad);
            case HAND_OFF:
                return UtilityFunctions.withinMargin(CoralArmConstants.stateMarginOfError,
                        CoralArmConstants.handOffSetPoint_rad, data.positionRad);
            case CORAL_PICKUP:
                return UtilityFunctions.withinMargin(CoralArmConstants.stateMarginOfError,
                        CoralArmConstants.coralPickUpSetPoint_rad, data.positionRad);
            case STOPPED:
                return UtilityFunctions.withinMargin(CoralArmConstants.stateMarginOfError, 0, data.velocityUnits); // Ensure
                                                                                                                   // velocity
                                                                                                                   // is
                                                                                                                   // near
                                                                                                                   // zero
                                                                                                                   // when
                                                                                                                   // stopped.
            default:
                return false; // Return false if the state is unrecognized.
        }
    }

    // SET FUNCTIONS

    /**
     * Sets the voltage to the arm motors. This directly controls the motor voltage.
     * 
     * @param volts The voltage to apply to the arm motors.
     */
    public void setVoltage(double volts) {
        armIO.setVoltage(volts);
    }

    /**
     * Sets the state of the arm (e.g., STOPPED, STOWED, etc.). This will move the
     * arm
     * to preset angles or stop it depending on the state.
     * 
     * @param state The new state for the arm.
     */
    public void setState(CoralArmConstants.ArmStates state) {
        this.state = (CoralArmConstants.ArmStates) state;
        switch (this.state) {
            case STOPPED:
                stop(); // Stop the arm if in STOPPED state.
                break;
            case STOWED:
                setGoal(CoralArmConstants.stowSetPoint_rad); // Set the goal to the stowed position.
                break;
            case CORAL_PICKUP:
                setGoal(CoralArmConstants.coralPickUpSetPoint_rad); // Set the goal to the coral pickup position.
                break;

            case HAND_OFF:
                setGoal(CoralArmConstants.handOffSetPoint_rad); // Set the goal to the hand-off position.
                break;

            default:
                stop(); // Stop the arm in any unrecognized state.
                break;
        }
    }

    public SysIdTuner getSysIdTuner() {
        return sysIdTuner;
    }

    private Angle getPitch() {
        return Angle.ofBaseUnits(data.positionRad + Units.degreesToRadians(-55), Radians); // remove offset once coral
                                                                                           // arm code is fixed
    }

    private Pose3d getPose3d() {
        Pose3d pose = new Pose3d(0, 0.35, 0.4,
                new Rotation3d(Angle.ofBaseUnits(0, Radians), getPitch(),
                        Angle.ofBaseUnits(Units.degreesToRadians(90), Radians)));
        return pose;
    }

    /**
     * Sets the target position for the arm's PID controller.
     * 
     * @param setpoint The desired target position for the arm in radians.
     */
    public void setGoal(double setpoint) {
        System.out.println(setpoint);
        profile.setGoal(setpoint); // Set the PID controller's goal.
    }

    // UTILITY FUNCTIONS

    /**
     * Stops the arm completely. This method is for use in emergencies or on
     * startup.
     */
    public void stop() {
        setVoltage(0); // Apply zero volts to stop the arm.
    }

    /**
     * Moves the arm to its goal using both PID control and feedforward
     * calculations.
     * This method combines PID and feedforward to control the arm's movement.
     */
    private void moveToGoal() {
        // Get the setpoint from the PID controller
        State firstState = profile.getSetpoint();

        // Calculate the PID control voltage based on the arm's current position
        profile.calculate(getPositionRad());

        State nextState = profile.getSetpoint(); // Get the next state of the setpoint

        // Calculate the feedforward voltage based on velocity
        double ffVoltage = feedforward.calculate(getPositionRad(), firstState.velocity);
        SmartDashboard.putNumber("FF", ffVoltage);
        // Apply the combined PID and feedforward voltages to the arm
        armIO.setPosition(firstState.position, ffVoltage);
        SmartDashboard.putNumber("goal coral", profile.getGoal().position);
        SmartDashboard.putNumber("setpoint pos coral", firstState.position);
        SmartDashboard.putNumber("setpoint vel coral", firstState.velocity);

    }

    // PERIODIC FUNCTIONS

    /**
     * Runs the logic for the current arm state. This is called periodically to
     * update the arm's behavior.
     */
    public void runState() {
        switch (state) {
            case STOPPED:
                stop(); // If the arm is stopped, we stop it.
                break;
            default:
                moveToGoal(); // In other states, move the arm to its goal position.
                break;
        }
    }

    /**
     * Logs the arm's data to Shuffleboard for monitoring. This is useful for
     * debugging and analysis.
     */
    private void logData() {
        // Log various arm parameters to Shuffleboard
        Logger.recordOutput("subsystems/arms/coralArm/Current Command",
                this.getCurrentCommand() == null ? "None" : this.getCurrentCommand().getName());
        Logger.recordOutput("subsystems/arms/coralArm/position", data.positionRad);
        Logger.recordOutput("subsystems/arms/coralArm/velocity", data.velocityUnits);
        Logger.recordOutput("subsystems/arms/coralArm/input volts", data.inputVolts);
        Logger.recordOutput("subsystems/arms/coralArm/applied volts", data.motorAppliedVolts);
        Logger.recordOutput("subsystems/arms/coralArm/current amps", data.motorCurrentAmps);
        Logger.recordOutput("subsystems/arms/coralArm/temperature", data.motorTempCelcius);

        // Update the visualization on the SmartDashboard with the arm's position
        armLigament.setAngle(Math.toDegrees(data.positionRad));

        Logger.recordOutput("subsystems/arms/coralArm/state", state.name());

        // Logger.recordOutput("zeropose", zeroedComponentPose);

        publisher.set(getPose3d());

        Logger.recordOutput("subsystems/arms/coralArm/coral arm mechanism", mechanism2d);
    }

    /** Periodic method for updating arm behavior. */
    @Override
    public void periodic() {

        armIO.updateData(data);

        runState();

        logData();

        motorData.get("arm_motor").position = data.positionRad;
        motorData.get("arm_motor").acceleration = data.accelerationUnits;
        motorData.get("arm_motor").velocity = data.velocityUnits;
        motorData.get("arm_motor").appliedVolts = data.appliedVolts;
    }
}
