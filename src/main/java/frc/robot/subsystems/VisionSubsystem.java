package frc.robot.subsystems;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.lib.camera.PhotonHelper;
import frc.robot.lib.camera.PhotonHelper.MeasurementProvider;
import frc.robot.lib.subsystems.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {
    private final PhotonHelper leftCamera;
    // private final PhotonHelper rightCamera;
    private final MeasurementProvider measurementProvider;
    private final Supplier<Pose2d> swervePose;
    private final StructPublisher<Pose2d> predictPose = NetworkTableInstance.getDefault()
        .getStructTopic("AdvantageScope/PredictPose", Pose2d.struct).publish();
    
    public VisionSubsystem(MeasurementProvider measurementProvider, Supplier<Pose2d> swervePose) {
        super("Vision", false);
        this.leftCamera = new PhotonHelper(
            "left-Cam",
            new Translation3d(0.159862, 0.134743, 0.225177),
            Units.degreesToRadians(-10.0), Units.degreesToRadians(-15.0));
        this.measurementProvider = measurementProvider;
        this.swervePose = swervePose;
    }

    @Override
    public void periodic() {
        this.leftCamera.updateFieldPose(this.measurementProvider, this.swervePose.get());      
        this.predictPose.accept(this.swervePose.get()); 
    }

    @Override
    public void putDashboard() {
    }
}
