package frc.robot.subsystems.vision.real;

import java.util.List;

import org.littletonrobotics.junction.Logger;
import org.photonvision.*;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.estimation.TargetModel;
import org.photonvision.targeting.*;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Robot;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionConstants.StandardDeviations;

public class Photonvision implements VisionIO {
    // * FROM VisionIO
    // PhotonPoseEstimator poseEstimatorList[] =
    // VisionConstants.CameraReal.poseEstimatorList;
    // PhotonCamera cameraList[] = VisionConstants.CameraReal.cameraList;
    private final PhotonCamera cam1 = new PhotonCamera("1");
    private final PhotonCamera cam2 = new PhotonCamera("2");
    private final PhotonCamera cam3 = new PhotonCamera("3");
    private final PhotonCamera cam4 = new PhotonCamera("4");
    private final PhotonCamera cam5 = new PhotonCamera("5");
    private final PhotonCamera cam6 = new PhotonCamera("6");

    private boolean disable3 = false;

    private final PhotonCamera[] cameraList = { cam1, cam2, cam3, cam4, cam5, cam6 };
    private PhotonPoseEstimator poseEstimatorList[] = VisionConstants.CameraReal.poseEstimatorList;

    private VisionData visionData;

    public Photonvision(VisionData visionData) {
        int index = 0;
        for (PhotonPoseEstimator poseEstimator : poseEstimatorList) {
            // Use MultiTag detection on the coprocessor, and fall back to the least
            // uncertain tag if that fails
            poseEstimator.setPrimaryStrategy(PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR);
            poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

            // redundant, but why not (setting the correct apriltag size/model and correct
            // field layout)
            poseEstimator.setTagModel(TargetModel.kAprilTag36h11);
            poseEstimator.setFieldTags(VisionConstants.aprilTagFieldLayout);
            index++;
        }

        this.visionData = visionData;
        for (int i = 0; i < VisionConstants.CameraReal.numCameras; i++) {
            logTarget(i);

        }
    }

    @Override
    public PhotonCamera getCamera(int index) {
        return cameraList[index];
    }

    @Override
    public void setDisable3(boolean disable) {
        disable3 = disable;
    }

    public void updatePose() {
        visionData.targetsSeenIds.clear();

        // Cam # minus 1
        // Cam 3 missing, cam 2 is bad because of mount droop

        // cameraUpdatePose(0);
        cameraUpdatePose(1);
        cameraUpdatePose(2);
        // cameraUpdatePose(3);
        // cameraUpdatePose(4);
        // cameraUpdatePose(5);
    }

    public void cameraUpdatePose(int index) {

        if (index == 1 || index == 2) {
            poseEstimatorList[index].addHeadingData(Timer.getTimestamp(), Robot.swerve.getRotation2d());
        }

        PhotonCamera camera = cameraList[index];
        PhotonPoseEstimator poseEstimator = poseEstimatorList[index];

        List<PhotonPipelineResult> pipelineResults = camera.getAllUnreadResults();

        // for each unused result in the pipeline
        for (PhotonPipelineResult pipelineResult : pipelineResults) {

            // skip if no tags found
            if (!pipelineResult.hasTargets()) {
                Logger.recordOutput("Vision/Cam" + (index + 1) + "/ No targets", true);
                logBlank(index);
                continue;
            }

            double latencyMillis = pipelineResult.metadata.getLatencyMillis();

            visionData.latencyMillis[index] = latencyMillis;
            visionData.targetsSeen[index] = pipelineResult.getTargets().size();

            for (var target : pipelineResult.getTargets()) {
                visionData.targetsSeenIds.add(target.getFiducialId());
            }

            // skip if latency is too high
            if (latencyMillis > VisionConstants.RejectionRequirements.maxLatencyMilliSec) {
                Logger.recordOutput("Vision/Cam" + (index + 1) + "/ High Latency", true);
                logBlank(index);

                continue;
            }

            if (pipelineResult.getTargets().size() == 1 &&
                    getHypotenuse(pipelineResult.getTargets().get(
                            0).bestCameraToTarget) > VisionConstants.RejectionRequirements.maxSingleTagDistanceMeters) {

                Logger.recordOutput("Vision/Cam" + (index + 1) + "/ single tag far", true);
                logBlank(index);

                continue;
            }
            visionData.distance[index] = getHypotenuse(pipelineResult.getTargets().get(
                    0).bestCameraToTarget);

            if (pipelineResult.getBestTarget().poseAmbiguity > 0.2 && pipelineResult.getTargets().size() == 1) {
                Logger.recordOutput("Vision/Cam" + (index + 1) + "/pose ambiguity rejection", true);
                logBlank(index);

            }

            poseEstimator.setReferencePose(getReferencePose());
            var optional_robotPose = poseEstimator.update(pipelineResult);

            if (optional_robotPose.isEmpty()) {
                Logger.recordOutput("Vision/Cam" + (index + 1) + "/ pose empty", true);
                logBlank(index);
                continue;
            }

            Pose3d robotPose = optional_robotPose.get().estimatedPose;
            Logger.recordOutput("Vision/Cam" + (index + 1) + "/pitch", robotPose.getRotation().getMeasureY());
            Logger.recordOutput("Vision/Cam" + (index + 1) + "/roll", robotPose.getRotation().getMeasureX());

            visionData.visionEstimatedPoses[index] = robotPose;

            updateStandardDeviations(pipelineResult, index);

            // if ((Robot.swerve.getIsOTF() || DriverStation.isAutonomous())
            // && Robot.elevator.getCurrentCommand().getName() == "ScoreL234" && !(index ==
            // 1 || index == 2)) {
            // continue;
            // }

            // double timestamp = Timer.getFPGATimestamp() - latencyMillis / 1000.0;
            double timestamp = pipelineResult.getTimestampSeconds();
            Robot.swerve.visionUpdateOdometry(robotPose.toPose2d(), timestamp);
            logTarget(index);
        }
    }

    public void logTarget(int index) {
        Logger.recordOutput("Vision/Cam" + (index + 1) + "/latency", visionData.latencyMillis[index]);
        Logger.recordOutput("Vision/Cam" + (index + 1) + "/targetsSeen", visionData.targetsSeen[index]);
        Logger.recordOutput("Vision/Cam" + (index + 1) + "/pose", visionData.visionEstimatedPoses[index]);
        Logger.recordOutput("Vision/Cam" + (index + 1) + "/distance", visionData.distance[index]);
        Logger.recordOutput("Vision/Cam" + (index + 1) + "/pose ambiguity rejection", false);

        Logger.recordOutput("Vision/Cam" + (index + 1) + "/ No targets", false);
        Logger.recordOutput("Vision/Cam" + (index + 1) + "/ High Latency", false);
        Logger.recordOutput("Vision/Cam" + (index + 1) + "/ pose empty", false);
    }

    public void logBlank(int index) {
        Logger.recordOutput("Vision/Cam" + (index + 1) + "/latency", -1.0);
        Logger.recordOutput("Vision/Cam" + (index + 1) + "/targetsSeen", 1);
        Logger.recordOutput("Vision/Cam" + (index + 1) + "/pose", new Pose3d());
    }

    public double getHypotenuse(Transform3d transform3d) {
        return Math.sqrt(
                Math.pow(transform3d.getX(), 2) + Math.pow(transform3d.getY(), 2) + Math.pow(transform3d.getZ(), 2));
    }

    public void updateStandardDeviations(PhotonPipelineResult result, int index) {
        SwerveDrivePoseEstimator poseEstimator = Robot.swerve.getPoseEstimator();

        if (result.getTargets().size() == 0) {
            return;
        }
        if (DriverStation.isDisabled()) {

            poseEstimator.setVisionMeasurementStdDevs(
                    VecBuilder.fill(StandardDeviations.PreMatch.xy, StandardDeviations.PreMatch.xy,
                            StandardDeviations.PreMatch.thetaRads));
        }
        if (result.getTargets().size() == 1) {
            double xyStdDev = StandardDeviations.OneTag.regression.apply(visionData.distance[index]);

            poseEstimator.setVisionMeasurementStdDevs(
                    VecBuilder.fill(xyStdDev, xyStdDev, StandardDeviations.OneTag.thetaRads));
        }
        if (result.getTargets().size() == 2) {
            poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(StandardDeviations.TwoTag.xy,
                    StandardDeviations.TwoTag.xy,
                    StandardDeviations.TwoTag.thetaRads));
        }
        if (result.getTargets().size() > 2) {
            poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(StandardDeviations.ManyTag.xy,
                    StandardDeviations.ManyTag.xy,
                    StandardDeviations.ManyTag.thetaRads));
        }
    }

    @Override
    public void setStrategyCam12(PoseStrategy strat) {
        // poseEstimatorList[1].setMultiTagFallbackStrategy(strat);
        // poseEstimatorList[2].setMultiTagFallbackStrategy(strat);

    }
}
