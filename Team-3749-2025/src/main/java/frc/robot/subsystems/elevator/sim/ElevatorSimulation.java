package frc.robot.subsystems.elevator.sim;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.LinearPlantInversionFeedforward;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.estimator.KalmanFilterLatencyCompensator;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.utils.MiscConstants.SimConstants;

/**
 * Simulation for elevator subsystem
 * 
 * @author Dhyan Soni
 */

public class ElevatorSimulation implements ElevatorIO {


    // a physics simulation of the system defined above
    private ElevatorSim elevatorSim;

    private double inputVolts = 0;
    private double previousVelocity = 0;
    private double velocity = 0;

    public ElevatorSimulation(LinearSystem<N2, N1, N2> elevatorSystem) {
        System.out.println("[Init] Creating ElevatorSimulation");
        elevatorSim = new ElevatorSim(
                elevatorSystem, // system
                DCMotor.getNEO(2), // motors used
                ElevatorConstants.ElevatorSpecs.minHeightMeters, // min height
                ElevatorConstants.ElevatorSpecs.maxHeightMeters, // max height
                true, // simulate gravity
                ElevatorConstants.ElevatorSpecs.minHeightMeters, // starting height
                0.00, // position standard deviation
                0.0 ); // velocity standard deviation
    }

    @Override
    public void updateData(ElevatorData data) {

        elevatorSim.update(0.02);
        previousVelocity = velocity;
        velocity = elevatorSim.getVelocityMetersPerSecond();
        data.positionMeters = elevatorSim.getPositionMeters();
        data.velocityMetersPerSecond = velocity;
        data.accelerationUnits = (velocity - previousVelocity) / SimConstants.loopPeriodSec;

        data.inputVolts = inputVolts;
        data.leftAppliedVolts = inputVolts;
        data.rightAppliedVolts = inputVolts;
        data.leftCurrentAmps = elevatorSim.getCurrentDrawAmps();
        data.rightCurrentAmps = data.leftCurrentAmps;

        // Sim has no temp
        data.leftTempCelcius = 0;
        data.rightTempCelcius = data.leftTempCelcius;
    }

    @Override
    public void setVoltage(double volts) {
        inputVolts = MathUtil.applyDeadband(inputVolts, 0.05);
        inputVolts = MathUtil.clamp(volts, -12, 12);
        elevatorSim.setInputVoltage(inputVolts);
    }
}
