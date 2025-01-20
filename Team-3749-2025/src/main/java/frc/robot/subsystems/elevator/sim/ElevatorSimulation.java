package frc.robot.subsystems.elevator.sim;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
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
    private double inputVolts = 0;
    private double previousVelocity = 0;
    private double velocity = 0;

    private final ElevatorSim elevatorSimSystem = new ElevatorSim(
            DCMotor.getNEO(2),
            ElevatorConstants.ElevatorSpecs.gearing,
            ElevatorConstants.ElevatorSpecs.carriageMassKg,
            ElevatorConstants.ElevatorSpecs.drumRadiusMeters,
            ElevatorConstants.ElevatorSpecs.minHeightMeters,
            ElevatorConstants.ElevatorSpecs.maxHeightMeters,
            true,
            ElevatorConstants.ElevatorSpecs.startingHeightMeters);


    LinearSystem<N2, N1, N2> elevatorSystem = LinearSystemId.identifyPositionSystem(2.3586, 0.22615);
    LinearQuadraticRegulator<N2, N1, N2> controller = new LinearQuadraticRegulator<>(elevatorSystem,
            VecBuilder.fill(0.01, 0.05), VecBuilder.fill(12), 0.02);


    public ElevatorSimulation() {
        System.out.println("[Init] Creating ElevatorSimulation");
        controller.latencyCompensate(elevatorSystem,0.02,0.025);
    }

    @Override
    public void updateData(ElevatorData data) {
        elevatorSimSystem.update(0.02);
        previousVelocity = velocity;
        velocity = elevatorSimSystem.getVelocityMetersPerSecond();
        data.positionMeters = elevatorSimSystem.getPositionMeters();
        data.velocityMetersPerSecond = velocity;
        data.accelerationUnits = (velocity - previousVelocity) / SimConstants.loopPeriodSec;

        data.inputVolts = inputVolts;
        data.leftAppliedVolts = inputVolts;
        data.rightAppliedVolts = inputVolts;
        data.leftCurrentAmps = elevatorSimSystem.getCurrentDrawAmps();
        data.rightCurrentAmps = data.leftCurrentAmps;

        // Sim has no temp
        data.leftTempCelcius = 0;
        data.rightTempCelcius = data.leftTempCelcius;
    }

    @Override
    public void setVoltage(double volts) {
        inputVolts = MathUtil.applyDeadband(inputVolts, 0.05);
        inputVolts = MathUtil.clamp(volts, -12, 12);
        elevatorSimSystem.setInputVoltage(inputVolts);
    }
}
