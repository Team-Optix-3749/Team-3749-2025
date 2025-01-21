package frc.robot.subsystems.elevator;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.LinearPlantInversionFeedforward;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorStates;
import frc.robot.subsystems.elevator.ElevatorIO.ElevatorData;
import frc.robot.subsystems.elevator.real.ElevatorSparkMax;
import frc.robot.subsystems.elevator.sim.ElevatorSimulation;
import frc.robot.utils.ShuffleData;
import frc.robot.utils.UtilityFunctions;

/**
 * Elevator subsystem
 * 
 * @author Dhyan Soni
 * @author Andrew Ge
 */

public class Elevator extends SubsystemBase {
    private ElevatorIO elevatorio;
    private ElevatorData data = new ElevatorData();
    private ElevatorStates state = ElevatorStates.STOP;

    /**
     * NX is represents a vector containing X numbers
     * The first NX is the number of states (ex. position and velocity would make
     * N2)
     * The second NX is the number of inputs (ex. Voltage would make N1)
     * The third NX is the number of outputs (ex. position and velocity)
     */

    // A system describing the elevator
    private LinearSystem<N2, N1, N2> elevatorSystem = LinearSystemId.identifyPositionSystem(
            ElevatorConstants.ElevatorControl.kVSim, // kV from SysId
            ElevatorConstants.ElevatorControl.kASim); // kA from SysId

    // a PID controller with values calculated based on desired error ranges
    private LinearQuadraticRegulator<N2, N1, N2> controller = new LinearQuadraticRegulator<>(
            elevatorSystem, // Linear System
            VecBuilder.fill(0.1, 0.5), // maximum desired error for positoin and velocity, respectively
            VecBuilder.fill(12), // maximum voltage that will be applied
            0.02); // discrete time step size

    // a filter to avoid mismeasurements
    private KalmanFilter<N2, N1, N2> filter = new KalmanFilter<>(
            Nat.N2(), // placeholder for 2 natural numbers: state number
            Nat.N2(), // placeholder for 2 natural numbers: output number
            elevatorSystem, // linear system
            VecBuilder.fill(0.01, 0.05), // std dev for states: how accurate is our model
            VecBuilder.fill(0.000005, 0.00001), // std dev for measurements: how good are our encoders
            0.02); // discrete time step size

    // a feedforward controller based on the system's dynamics
    private LinearPlantInversionFeedforward<N2, N1, N2> feedforward = new LinearPlantInversionFeedforward<>(
            elevatorSystem,
            0.02);

    // all of the above combined to one control loop
    private LinearSystemLoop<N2, N1, N2> controlLoop;

    // utilized only for the Motion Profile
    // private ProfiledPIDController pidController = new ProfiledPIDController(
    // 0,
    // 0,
    // 0,
    // new Constraints(ElevatorConstants.ElevatorControl.maxV,
    // ElevatorConstants.ElevatorControl.maxA));

    private ShuffleData<String> currentCommandLog = new ShuffleData<String>(this.getName(), "current command", "None");
    private ShuffleData<Double> positionMetersLog = new ShuffleData<Double>("Elevator", "position meters", 0.0);
    private ShuffleData<Double> velocityUnitsLog = new ShuffleData<Double>("Elevator", "velocity units", 0.0);
    private ShuffleData<Double> accelerationUnitsLog = new ShuffleData<Double>("Elevator", "acceleration units", 0.0);
    private ShuffleData<Double> inputVoltsLog = new ShuffleData<Double>("Elevator", "input volts", 0.0);
    private ShuffleData<Double> leftAppliedVoltsLog = new ShuffleData<Double>("Elevator", "left applied volts", 0.0);
    private ShuffleData<Double> rightAppliedVoltsLog = new ShuffleData<Double>("Elevator", "right applied volts", 0.0);
    private ShuffleData<Double> leftCurrentAmpsLog = new ShuffleData<Double>("Elevator", "left current amps", 0.0);
    private ShuffleData<Double> rightCurrentAmpsLog = new ShuffleData<Double>("Elevator", "right current amps", 0.0);
    private ShuffleData<Double> leftTempCelciusLog = new ShuffleData<Double>("Elevator", "left temp celcius", 0.0);
    private ShuffleData<Double> rightTempCelciusLog = new ShuffleData<Double>("Elevator", "right temp celcius", 0.0);

    // For tuning on real
    // private ShuffleData<Double> kPData = new ShuffleData<Double>("Elevator",
    // "kPData", ElevatorConstants.ElevatorControl.kPSim);
    // private ShuffleData<Double> kDData = new ShuffleData<Double>("Elevator",
    // "kDData", ElevatorConstants.ElevatorControl.kDSim);
    // private ShuffleData<Double> kGData = new ShuffleData<Double>("Elevator",
    // "kGData", ElevatorConstants.ElevatorControl.kGSim);
    // private ShuffleData<Double> kVData = new ShuffleData<Double>("Elevator",
    // "kVData", ElevatorConstants.ElevatorControl.kVSim);
    // private ShuffleData<Double> kAData = new ShuffleData<Double>("Elevator",
    // "kAData", ElevatorConstants.ElevatorControl.kASim);

    private Mechanism2d mech = new Mechanism2d(3, 3);
    private MechanismRoot2d root = mech.getRoot("elevator", 2, 0);
    private MechanismLigament2d elevatorMech = root
            .append(new MechanismLigament2d("elevator", ElevatorConstants.ElevatorSpecs.baseHeight, 90));

    public Elevator() {
        controller.latencyCompensate(elevatorSystem, 0.02, 0.025);
        controlLoop = new LinearSystemLoop<>(controller, feedforward, filter, 12);

        if (Robot.isSimulation()) {
            elevatorio = new ElevatorSimulation(elevatorSystem);
        } else {
            elevatorio = new ElevatorSparkMax();
        }

        System.out.println(controlLoop.toString());
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

    // returns true when the state is reached
    public boolean getIsStableState() {
        switch (state) {
            case L1:
                return UtilityFunctions.withinMargin(0.01, ElevatorConstants.StateHeights.l1Height,
                        data.positionMeters);
            case L2:
                return UtilityFunctions.withinMargin(0.01, data.positionMeters,
                        ElevatorConstants.StateHeights.l2Height);
            case L3:
                return UtilityFunctions.withinMargin(0.01, data.positionMeters,
                        ElevatorConstants.StateHeights.l3Height);
            case L4:
                return UtilityFunctions.withinMargin(0.01, data.positionMeters,
                        ElevatorConstants.StateHeights.l4Height);
            default:
                return false;
        }
    }

    public void setVoltage(double volts) {
        elevatorio.setVoltage(volts);
    }

    public void setState(ElevatorStates state) {
        System.out.println("SET STATE");
        this.state = state;
        switch (state) {
            case STOP:
                runStateStop();
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
                setGoal(6);
                break;
            case STOW:
            default:
                setGoal(0);
                break;
        }
    }

    public void setGoal(double height) {
        controlLoop.setNextR(VecBuilder.fill(height, 0));
    }

    private void runState() {
        switch (state) {
            case STOP:
                runStateStop();
                break;
            default:
                moveToGoal();
                break;
        }
    }

    private void moveToGoal() {
        // just to advance the motion profile
        controlLoop.predict(0.02);
        elevatorio.setVoltage(controlLoop.getU(0));
    }

    private void runStateStop() {
        stop();
    }

    public void stop() {
        elevatorio.setVoltage(0);
    }

    private void logData() {
        currentCommandLog.set(this.getCurrentCommand() == null ? "None" : this.getCurrentCommand().getName());
        positionMetersLog.set(data.positionMeters);
        velocityUnitsLog.set(data.velocityMetersPerSecond);
        accelerationUnitsLog.set(data.accelerationUnits);
        inputVoltsLog.set(data.inputVolts);
        leftAppliedVoltsLog.set(data.leftAppliedVolts);
        rightAppliedVoltsLog.set(data.rightAppliedVolts);
        leftCurrentAmpsLog.set(data.leftCurrentAmps);
        rightCurrentAmpsLog.set(data.rightCurrentAmps);
        leftTempCelciusLog.set(data.leftTempCelcius);
        rightTempCelciusLog.set(data.rightTempCelcius);

        elevatorMech.setLength(ElevatorConstants.ElevatorSpecs.baseHeight + data.positionMeters);

        SmartDashboard.putString("State", state.name());

        SmartDashboard.putData("elevator mechanism", mech);
    }

    private void updateData() {
        elevatorio.updateData(data);
        controlLoop.correct(VecBuilder.fill(data.positionMeters, data.velocityMetersPerSecond));
        ;
    }

    @Override
    public void periodic() {
        updateData();
        runState();
        logData();
        // pidController.setPID(kPData.get(),0,kDData.get())
    }
}
