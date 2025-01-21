package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.arm.ArmIO.ArmData;
import frc.robot.utils.ShuffleData;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;

/**
 * Arm parent class to be inherited by the other arms
 * 
 * @author Akhil
 * @author Weston Gardner
 */

public abstract class Arm extends SubsystemBase {

    protected ArmIO armIO;
    protected ArmData data = new ArmData();
    protected ShuffleData<String> currentCommandLog = new ShuffleData<>(this.getName(), "current command", "None");
    protected ShuffleData<Double> positionUnitsLog = new ShuffleData<>(this.getName(), "position units", 0.0);
    protected ShuffleData<Double> velocityUnitsLog = new ShuffleData<>(this.getName(), "velocity units", 0.0);
    protected ShuffleData<Double> appliedVoltsLog = new ShuffleData<>(this.getName(), "applied volts", 0.0);
    protected ShuffleData<Double> currentAmpsLog = new ShuffleData<>(this.getName(), "current amps", 0.0);
    protected ShuffleData<Double> tempCelciusLog = new ShuffleData<>(this.getName(), "temp celcius", 0.0);
    public ShuffleData<Double> inputVoltsLog = new ShuffleData<Double>(this.getName(), "input volts", 0.0);

    protected Mechanism2d mechanism2d = new Mechanism2d(60, 60);
    protected MechanismRoot2d armRoot = mechanism2d.getRoot("ArmRoot", 30, 30);
    protected MechanismLigament2d armLigament = armRoot.append(new MechanismLigament2d("Arm", 24, 0));

    public Arm() {
    }

    // Method to set the voltage for the arm
    public void setVoltage(double volts) {
        armIO.setVoltage(volts);
    }

    // Method to update arm data
    protected void updateData() {
        armIO.updateData(data);
    }

    public double getPositionRad() {
        return data.positionUnits;
    }

    public double getVelocityRadPerSec() {
        return data.velocityUnits;
    }

    public abstract void setState(Enum<?> state);

    public abstract void stop();
}
