package frc.robot.lib.simulation;


import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;

public class AprilTagsSim {
    private final AprilTagFieldLayout layout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
    private Pose3d[] tagPoses = new Pose3d[this.layout.getTags().size()];
    private final StructArrayPublisher<Pose3d> aprilTagsPublisher = NetworkTableInstance.getDefault()
        .getStructArrayTopic("AprilTags", Pose3d.struct).publish();
    
    public AprilTagsSim() {
        for (int i = 0; i < this.layout.getTags().size(); i++) {
            this.tagPoses[i] = this.layout.getTagPose(i + 1).get();
        }
        this.aprilTagsPublisher.accept(this.tagPoses);
    }
}
