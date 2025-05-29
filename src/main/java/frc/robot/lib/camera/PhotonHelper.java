package frc.robot.lib.camera;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;

public class PhotonHelper extends PhotonCamera {
    private final AprilTagFieldLayout layout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
    private final Transform3d cameraPose;
    public final PhotonPoseEstimator poseEstimator;

    public PhotonHelper(
        String camName,
        Transform3d cameraPose
    ) {
        super(camName);
        this.cameraPose = cameraPose;
        this.poseEstimator = new PhotonPoseEstimator(this.layout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, this.cameraPose);
        this.poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    }

    public Transform3d getRobotToTagPose() {
        var result = this.getLatestResult();
        if (result == null) return new Transform3d();
        Transform3d cameraToTag = result.getBestTarget().getBestCameraToTarget();
        Transform3d robotToTag = this.cameraPose.plus(cameraToTag);
        return robotToTag;
    }
    
    public Pose3d getRobotToField() {
        Transform3d robotToTag = this.getRobotToTagPose();
        if (robotToTag == new Transform3d()) return new Pose3d();
        var result = this.getLatestResult();
        if (result == null) return new Pose3d();
        Pose3d fieldToTag = this.layout.getTagPose(result.getBestTarget().getFiducialId()).get();
        return fieldToTag.transformBy(robotToTag.inverse());
    }
}
