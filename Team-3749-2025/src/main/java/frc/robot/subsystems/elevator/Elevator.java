package frc.robot.subsystems.elevator;

import java.util.Map;
import java.util.function.Consumer;
import static edu.wpi.first.units.Units.Radians;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Robot;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorStates;
import frc.robot.subsystems.elevator.ElevatorIO.ElevatorData;
import frc.robot.subsystems.elevator.real.ElevatorSparkMax;
import frc.robot.subsystems.elevator.sim.ElevatorSimulation;
import frc.robot.utils.LoggedTunableNumber;
import frc.robot.utils.ShuffleData;
import frc.robot.utils.SysIdTuner;
import frc.robot.utils.MotorData;
import frc.robot.utils.UtilityFunctions;

import static edu.wpi.first.units.Units.*;

/**
 * Elevator subsystem
 * 
 * @author Dhyan Soni
 * @author Andrew Ge
 */
@SuppressWarnings("unused")
public class Elevator extends SubsystemBase {
    private ElevatorIO elevatorio;
    private ElevatorData data = new ElevatorData();
    private ElevatorStates state = ElevatorStates.STOP;

    static Consumer<Voltage> setVolts;
    static Consumer<SysIdRoutineLog> log;

    private ProfiledPIDController profile = new ProfiledPIDController(
            0,
            0,
            0,
            new TrapezoidProfile.Constraints(ElevatorConstants.ElevatorControl.maxV,
                    ElevatorConstants.ElevatorControl.maxA));

    private ElevatorFeedforward feedforward = new ElevatorFeedforward(
            ElevatorConstants.ElevatorControl.kS,
            ElevatorConstants.ElevatorControl.kG,
            ElevatorConstants.ElevatorControl.kV,
            ElevatorConstants.ElevatorControl.kA);

    private ShuffleData<String> currentCommandLog = new ShuffleData<String>(this.getName(), "current command", "None");
    private LoggedTunableNumber positionMetersLog = new LoggedTunableNumber("Elevator/position", 0.0);
    private LoggedTunableNumber velocityMetersPerSecLog = new LoggedTunableNumber("Elevator/velocity", 0.0);
    private LoggedTunableNumber accelerationMetersPerSecSquaredLog = new LoggedTunableNumber("Elevator/acceleration",
            0.0);

    private LoggedTunableNumber inputVoltsLog = new LoggedTunableNumber("Elevator/input volts", 0.0);
    private LoggedTunableNumber leftAppliedVoltsLog = new LoggedTunableNumber("Elevator/left applied volts", 0.0);
    private LoggedTunableNumber rightAppliedVoltsLog = new LoggedTunableNumber("Elevator/right applied volts", 0.0);
    private LoggedTunableNumber leftCurrentAmpsLog = new LoggedTunableNumber("Elevator/left current amps", 0.0);
    private LoggedTunableNumber rightCurrentAmpsLog = new LoggedTunableNumber("Elevator/right current amps", 0.0);
    private LoggedTunableNumber leftTempCelciusLog = new LoggedTunableNumber("Elevator/left temp celcius", 0.0);
    private LoggedTunableNumber rightTempCelciusLog = new LoggedTunableNumber("Elevator/right temp celcius", 0.0);

    private LoggedTunableNumber setpointVelocityLog = new LoggedTunableNumber("Elevator/setpoint velocity", 0.0);
    private LoggedTunableNumber setpointPositionLog = new LoggedTunableNumber("Elevator/setpoint position", 0.0);

    private LoggedTunableNumber kPData = new LoggedTunableNumber("Elevator/kP",
            ElevatorConstants.ElevatorControl.kP);
    private LoggedTunableNumber kDData = new LoggedTunableNumber("Elevator/kD",
            ElevatorConstants.ElevatorControl.kD);
    private LoggedTunableNumber kVData = new LoggedTunableNumber("Elevator/kV",
            ElevatorConstants.ElevatorControl.kV);
    private LoggedTunableNumber kAData = new LoggedTunableNumber("Elevator/kA",
            ElevatorConstants.ElevatorControl.kA);
    private LoggedTunableNumber kGData = new LoggedTunableNumber("Elevator/kG",
            ElevatorConstants.ElevatorControl.kG);
    private LoggedTunableNumber kSData = new LoggedTunableNumber("Elevator/kS",
            ElevatorConstants.ElevatorControl.kS);
    private LoggedTunableNumber kIData = new LoggedTunableNumber("Elevator/kI",
            ElevatorConstants.ElevatorControl.kI);
    private LoggedTunableNumber maxVData = new LoggedTunableNumber("Elevator/maxV",
            ElevatorConstants.ElevatorControl.maxV);
    private LoggedTunableNumber maxAData = new LoggedTunableNumber("Elevator/maxA",
            ElevatorConstants.ElevatorControl.maxA);

    private Mechanism2d mech = new Mechanism2d(3, 3);
    private MechanismRoot2d root = mech.getRoot("elevator", 1, 0);
    private MechanismLigament2d elevatorMech = root
            .append(new MechanismLigament2d("elevator", ElevatorConstants.ElevatorSpecs.baseHeight, 90));

    // SysID
    Map<String, MotorData> motorData = Map.of(
            "elevator_motor", new MotorData(
                    (data.leftAppliedVolts + data.rightAppliedVolts) / 2.0,
                    data.positionMeters,
                    data.velocityMetersPerSecond,
                    data.accelerationMetersPerSecondSquared));

    private SysIdRoutine.Config config = new SysIdRoutine.Config(
            Volts.per(Seconds).of(1), // Voltage ramp rate
            Volts.of(7), // Max voltage
            Seconds.of(4) // Test duration
    );

    private SysIdTuner sysIdTuner;
    private double elevatorInnerStagePos;
    private double elevatorMiddleStagePos;

    StructPublisher<Pose3d> elevatorInnerStage = NetworkTableInstance.getDefault()
            .getStructTopic("Elevator Inner Stage", Pose3d.struct).publish();
    StructPublisher<Pose3d> elevatorMiddleStage = NetworkTableInstance.getDefault()
            .getStructTopic("Elevator Middle Stage", Pose3d.struct).publish();

    public Elevator() {
        if (Robot.isSimulation()) {
            elevatorio = new ElevatorSimulation();
        } else {
            elevatorio = new ElevatorSparkMax();
        }
        sysIdTuner = new SysIdTuner("elevator", config, this, elevatorio::setVoltage, motorData);
    }

    public SysIdTuner getSysIdTuner() {
        return sysIdTuner;
    }

    public ElevatorStates getState() {
        return state;
    }

    public double getPositionMeters() {
        return data.positionMeters;
    }

    public double getVelocityRadPerSec() {
        return data.velocityMetersPerSecond;
    }

    /** returns true when the state is reached */
    public boolean getIsStableState() {
        switch (state) {
            case L1:
                return UtilityFunctions.withinMargin(0.001, ElevatorConstants.StateHeights.l1Height,
                        data.positionMeters);
            case L2:
                return UtilityFunctions.withinMargin(0.001, data.positionMeters,
                        ElevatorConstants.StateHeights.l2Height);
            case L3:
                return UtilityFunctions.withinMargin(0.001, data.positionMeters,
                        ElevatorConstants.StateHeights.l3Height);
            case L4:
                return UtilityFunctions.withinMargin(0.001, data.positionMeters,
                        ElevatorConstants.StateHeights.l4Height);
            default:
                return false;
        }
    }

    public void setVoltage(double volts) {
        elevatorio.setVoltage(volts);
    }

    public void setState(ElevatorStates state) {
        this.state = state;
        switch (state) {
            case STOP:
                stop();
                break;
            case L1:
                setGoal(ElevatorConstants.StateHeights.l1Height);
                break;
            case L2:
                setGoal(ElevatorConstants.StateHeights.l2Height);
                break;
            case L3:
                setGoal(ElevatorConstants.StateHeights.l3Height);
                break;
            case L4:
                setGoal(ElevatorConstants.StateHeights.l4Height);
                break;
            case MAX:
                setGoal(ElevatorConstants.ElevatorSpecs.maxHeightMeters);
                break;
            case STOW:
                setGoal(ElevatorConstants.ElevatorSpecs.baseHeight);
                break;
            default:
                setGoal(0);
                break;
        }
    }

    public void setGoal(double height) {
        profile.setGoal(height);
    }

    private void runState() {
        switch (state) {
            case STOP:
                stop();
                break;
            default:
                moveToGoal();
                break;
        }
    }

    private void moveToGoal() {
        State firstState = profile.getSetpoint();
        profile.calculate(getPositionMeters());

        State nextState = profile.getSetpoint();
        double ffVoltage = feedforward.calculate(firstState.velocity, nextState.velocity);

        elevatorio.setPosition(firstState.position, ffVoltage);
    }

    public void stop() {
        elevatorio.setVoltage(0);
    }

    private void logData() {
        Logger.recordOutput("subystems/elevator/Current Command", this.getCurrentCommand() == null ? "None" : this.getCurrentCommand().getName());

        Logger.recordOutput("subystems/elevator/postion", data.positionMeters);
        Logger.recordOutput("subystems/elevator/velocity", data.velocityMetersPerSecond);

        Logger.recordOutput("subystems/elevator/acceleration", data.accelerationMetersPerSecondSquared);

        leftAppliedVoltsLog.set(data.leftAppliedVolts);
        rightAppliedVoltsLog.set(data.rightAppliedVolts);
        leftCurrentAmpsLog.set(data.leftCurrentAmps);
        rightCurrentAmpsLog.set(data.rightCurrentAmps);
        leftTempCelciusLog.set(data.leftTempCelcius);
        rightTempCelciusLog.set(data.rightTempCelcius);

        setpointVelocityLog.set(profile.getSetpoint().velocity);
        setpointPositionLog.set(profile.getSetpoint().position);

        elevatorMech.setLength(ElevatorConstants.ElevatorSpecs.baseHeight + data.positionMeters);
        SmartDashboard.putData("elevator mechanism", mech);

        ElevatorConstants.ElevatorControl.kP = kPData.get();
        ElevatorConstants.ElevatorControl.kD = kDData.get();
        ElevatorConstants.ElevatorControl.kV = kVData.get();
        ElevatorConstants.ElevatorControl.kA = kAData.get();
        ElevatorConstants.ElevatorControl.kG = kGData.get();
        ElevatorConstants.ElevatorControl.kS = kSData.get();
        ElevatorConstants.ElevatorControl.kI = kIData.get();
        ElevatorConstants.ElevatorControl.maxV = maxVData.get();
        ElevatorConstants.ElevatorControl.maxA = maxAData.get();
        elevatorInnerStagePos = data.positionMeters / 2;
        elevatorMiddleStagePos = data.positionMeters - Units.inchesToMeters(1);
        elevatorInnerStage.set(new Pose3d(getTransform3d(elevatorInnerStagePos).getTranslation(),
                getTransform3d(elevatorInnerStagePos).getRotation()));
        elevatorMiddleStage.set(new Pose3d(getTransform3d(elevatorMiddleStagePos).getTranslation(),
                getTransform3d(elevatorMiddleStagePos).getRotation()));
    }

    private Transform3d getTransform3d(double pos) {
        Transform3d transform = new Transform3d(0, 0, pos, new Rotation3d(Angle.ofBaseUnits(0, Radians),
                Angle.ofBaseUnits(0, Radians), Angle.ofBaseUnits(0, Radians)));
        return transform;
    }

    @Override
    public void periodic() {
        elevatorio.updateData(data);
        runState();
        logData();

        motorData.get("elevator_motor").position = data.positionMeters;
        motorData.get("elevator_motor").acceleration = data.accelerationMetersPerSecondSquared;
        motorData.get("elevator_motor").velocity = data.velocityMetersPerSecond;
        motorData.get("elevator_motor").appliedVolts = (data.leftAppliedVolts + data.rightAppliedVolts) / 2.0;

        // Map<String, MotorData> motorData = Map.of(
        // "elevator_motor", new MotorData(
        // (data.leftAppliedVolts + data.rightAppliedVolts) / 2.0,
        // data.positionMeters,
        // data.velocityMetersPerSecond,
        // data.accelerationUnits));

        // sysIdTuner = new SysIdTuner("elevator", config, this, elevatorio::setVoltage,
        // motorData);
        // pidController.setPID(kPData.get(),0,kDData.get())
    }
}
