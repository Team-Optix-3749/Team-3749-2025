package frc.robot.subsystems.vision;

import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.vision.VisionIO.*;
import frc.robot.subsystems.vision.real.Photonvision;
import frc.robot.subsystems.vision.sim.VisionSim;

public class Vision extends SubsystemBase {
    @SuppressWarnings("unused")
    private VisionIO visionIO;
    public VisionData visionData = new VisionData();

    public Vision() {
        if (Robot.isReal()) {
            visionIO = new Photonvision(visionData);
        } else {
            visionIO = new VisionSim();
        }

    }

    @Override
    public void periodic() {
        visionIO.updatePose();
    }

    public void setStrategyCam12(PoseStrategy strat) {
        visionIO.setStrategyCam12(strat);
    }

    public void disable3(boolean disable){
        visionIO.setDisable3(disable);
    }
}
