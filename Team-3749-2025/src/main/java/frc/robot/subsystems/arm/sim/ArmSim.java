package frc.robot.subsystems.arm.sim;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.subsystems.arm.ArmIO;
import frc.robot.utils.MiscConstants.SimConstants;

/**
 * IO implementation for an arm subsystem's simulation
 * 
 * @author Weston Gardner
 */

public class ArmSim implements ArmIO {

    private SingleJointedArmSim armSim;

    public ArmSim(
            int numMotors,
            double gearing,
            double momentOfInertia,
            double length_meters,
            double minAngle_degrees,
            double maxAngle_degrees,
            boolean simulateGravity,
            double startingAngle_Degrees) {

        System.out.println("[Init] Creating ArmSim");

        armSim = new SingleJointedArmSim(
                DCMotor.getNEO(numMotors),
                gearing,
                momentOfInertia,
                length_meters,
                minAngle_degrees * Math.PI / 180,
                maxAngle_degrees * Math.PI / 180,
                simulateGravity,
                startingAngle_Degrees * Math.PI / 180);

    }

    private double appliedVolts = 0;
    private double previousVelocity = 0;
    private double velocity = 0;
    private double conversionFactor = 1;

    /**
     * Updates the set of loggable inputs for the sim.
     * 
     * @param data
     */

    @Override
    public void updateData(ArmData data) {
        armSim.update(SimConstants.loopPeriodSec);

        previousVelocity = velocity;
        velocity = armSim.getVelocityRadPerSec() * conversionFactor;
        data.positionUnits = armSim.getAngleRads() * conversionFactor; // Directly use the angle from the simulation
        data.velocityUnits = velocity;
        data.accelerationUnits = (velocity - previousVelocity) / SimConstants.loopPeriodSec;
        data.currentAmps = armSim.getCurrentDrawAmps();
        data.inputVolts = appliedVolts;
        data.appliedVolts = appliedVolts;

        data.tempCelcius = 0.0; // sim has no temperature
    }

    /**
     * Run the motor at the specified voltage.
     * 
     * @param volts
     */

    @Override
    public void setVoltage(double volts) {
        appliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
        armSim.setInputVoltage(appliedVolts);
    }
}
