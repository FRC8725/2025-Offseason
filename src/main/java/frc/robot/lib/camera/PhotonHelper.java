package frc.robot.lib.camera;

import java.util.Optional;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class PhotonHelper {
    private final PhotonCamera camera;
    private final AprilTagFieldLayout layout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
    private final Transform3d cameraPose;
    private final PhotonPoseEstimator estimator;

    public PhotonHelper(
        String camName,
        Translation3d cameraTranslation,
        double pitch, double yaw
    ) {
        this.camera = new PhotonCamera(camName);
        this.cameraPose = new Transform3d(cameraTranslation, new Rotation3d(0.0, pitch, yaw));
        this.estimator = new PhotonPoseEstimator(this.layout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, this.cameraPose);
        this.estimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    }

    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        this.estimator.setReferencePose(prevEstimatedRobotPose);
        PhotonPipelineResult result = (PhotonPipelineResult) this.camera.getLatestResult();
        if (result == null) return null;
        return this.estimator.update(result);
    }

    public void updateFieldPose(MeasurementProvider measurementProvider, Pose2d swervePose) {
        Optional<EstimatedRobotPose> poseEstimate = this.getEstimatedGlobalPose(swervePose);
        if (poseEstimate.isPresent()) {
            SmartDashboard.putString("Camera/EstimatedPose", poseEstimate.get().estimatedPose.toPose2d().toString());
            Pose3d cameraPose = poseEstimate.get().estimatedPose;
            Transform3d caemraToRobot = this.cameraPose.inverse();

            measurementProvider.addVisionMeasurement(cameraPose.transformBy(caemraToRobot).toPose2d(), poseEstimate.get().timestampSeconds);
        }
    }
    
    public interface MeasurementProvider {
        public void addVisionMeasurement(Pose2d visionMeasurement, double timestampSeconds);
    }
}