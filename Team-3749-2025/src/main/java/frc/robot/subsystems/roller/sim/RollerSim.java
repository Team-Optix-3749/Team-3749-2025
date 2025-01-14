package frc.robot.subsystems.roller.sim;

import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.subsystems.roller.RollerIO;

public class RollerSim implements RollerIO {

    private FlywheelSim rollerMotor;

    public RollerSim() {
        DCMotor motor = DCMotor.getNEO(1);
        double gearRatio = 1.0;
        double momentOfInertia = 0.04;
        double measurementNoise = 0.0;

        LinearSystem<N1, N1, N1> flyWheelSystem = LinearSystemId.createFlywheelSystem(motor, gearRatio, momentOfInertia);
        rollerMotor = new FlywheelSim(flyWheelSystem, motor, measurementNoise);
    }
    
}
