//! maybe just remove this? whats the point in simming cameras

package frc.robot.subsystems.vision;

import java.util.ArrayList;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.Robot;
import frc.robot.subsystems.vision.VisionConstants.CameraReal;

public interface VisionIO {

    public static class VisionData {
        public Pose3d[] visionEstimatedPoses = {
                new Pose3d(), new Pose3d(), new Pose3d(), new Pose3d(), new Pose3d(), new Pose3d() };
        public double[] latencyMillis = new double[CameraReal.numCameras];
        public double[] distance = new double[CameraReal.numCameras];
        public int[] targetsSeen = new int[CameraReal.numCameras];
        public ArrayList<Integer> targetsSeenIds = new ArrayList<>();
    }

    // public void updateData(VisionData data);

    public void updatePose();

    public void cameraUpdatePose(int index);

    public default Pose2d getReferencePose() {
        return Robot.swerve.getPose();
    }

    public default PhotonCamera getCamera(int index) {
        return new PhotonCamera("" + index);
    }

    public default void setStrategyCam12(PoseStrategy strat){
        
    }
    public default void setDisable3(boolean disable){}

}
