package frc.robot.subsystems.roller.sim;

import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.subsystems.roller.RollerConstants;
import frc.robot.subsystems.roller.RollerIO;

public class RollerSim implements RollerIO {
    private FlywheelSim rollerMotor;

    public RollerSim() {
        DCMotor motor = DCMotor.getNEO(1);
        // calculates how angular velocity changes over time with applied voltage
        LinearSystem<N1, N1, N1> flyWheelSystem = LinearSystemId.createFlywheelSystem(motor, RollerConstants.momentOfInertia, RollerConstants.gearRatio);
        rollerMotor = new FlywheelSim(flyWheelSystem, motor, RollerConstants.measurementNoise);
    }
    
    @Override
    public void updateData(RollerData data) {
        rollerMotor.update(0.02);
        data.rollerAppliedVolts = rollerMotor.getInputVoltage();
        data.rollerVelocityRadPerSec = rollerMotor.getAngularVelocityRadPerSec();
        data.currentAmps = rollerMotor.getCurrentDrawAmps();
        data.rollerTempCelcius = 0.0;
        data.rollerPositionRotations += (data.rollerVelocityRadPerSec * 0.02 / (2 * Math.PI));
    }

    @Override
    public void setVoltage(double rollerVolts) {
        rollerMotor.setInputVoltage(rollerVolts);
    }
}
