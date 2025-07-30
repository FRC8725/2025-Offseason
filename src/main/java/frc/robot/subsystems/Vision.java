package frc.robot.subsystems;

import java.util.List;
import java.util.Objects;
import java.util.Set;
import java.util.stream.Collectors;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.lib.camera.PhotonHelper;

public class Vision extends SubsystemBase {
    // ---------- Object ---------- //
    private final Set<Integer> blockIds = Set.of(4, 5, 14, 15); // Barge Ids
    private final PhotonHelper[] cameras = new PhotonHelper[] {
        new PhotonHelper("FrontRight", Constants.Vision.FRONT_RIGHT),
        new PhotonHelper("BackRight", Constants.Vision.BACK_RIGHT)
        // new PhotonHelper("FrontRight", Const)
    };

    // ---------- Function ---------- //
    public boolean isInsideField(Translation2d translation2d) {
        if (translation2d.getX() >= 0.0 && translation2d.getX() <= Constants.Field.FIELD_X_SIZE &&
            translation2d.getY() >= 0.0 && translation2d.getY() <= Constants.Field.FIELD_Y_SIZE) {
            return true;
        }
        return false;
    }

    public boolean isAllConnected() {
        for (PhotonHelper camera : this.cameras) {
            if (!camera.isConnected()) return false;
        }
        return true;
    }

    public boolean removeResult(PhotonPipelineResult res) {
        for (PhotonTrackedTarget target : res.getTargets()) {
            if (this.blockIds.contains(target.getFiducialId())) return true;
        }
        return false;
    }

    // ---------- Function ---------- //
    public void addVisionMeasurements(SwerveDrivePoseEstimator poseEstimator) {
        for (PhotonHelper camera : this.cameras) {
            List<PhotonPipelineResult> results = camera.getAllUnreadResults().stream()
                .filter(result -> !removeResult(result))
                .collect(Collectors.toList());
            List<EstimatedRobotPose> estimatedPoses = results.stream()
                .map(result -> camera.poseEstimator.update(result).orElse(null))
                .filter(Objects::nonNull)
                .collect(Collectors.toList());
            
            for (EstimatedRobotPose pose : estimatedPoses) {
                if (this.isInsideField(pose.estimatedPose.getTranslation().toTranslation2d()) &&
                    (pose.strategy == PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR ||
                        (pose.strategy == PoseStrategy.LOWEST_AMBIGUITY &&
                            pose.targetsUsed.get(0).poseAmbiguity < 0.05 &&
                            pose.targetsUsed.get(0).area > 0.25))) {
                    poseEstimator.addVisionMeasurement(pose.estimatedPose.toPose2d(), pose.timestampSeconds);
                }
            }
        }
    }

    @Override
    public void periodic() {
        this.addVisionMeasurements(Swerve.getInstance().poseEstimator);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        // builder.addStringProperty("BackLeft", () -> this.cameras[1].getRobotToTagPose().getTranslation().toString(), null);
        for (PhotonHelper camera : this.cameras) {
            builder.addBooleanProperty(camera.getName() + " status", () -> camera.isConnected(), null);
        }
    }
}
