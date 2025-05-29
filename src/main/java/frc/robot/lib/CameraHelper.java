package frc.robot.lib;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Transform3d;

public class CameraHelper extends PhotonCamera {
    public final PhotonPoseEstimator poseEstimator;

    public CameraHelper(String name, Transform3d robotToCam) {
        super(name);
        this.poseEstimator = new PhotonPoseEstimator(
            AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape),
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            robotToCam);
        this.poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    }
}
