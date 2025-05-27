package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import frc.robot.SwerveConstants;
import frc.robot.lib.subsystems.SubsystemBase;

public class SwerveSim extends SubsystemBase {
    private Pose2d simPose = new Pose2d();
    private SwerveModuleState[] lastDesiredState;
    private final StructPublisher<Pose2d> simSwervePose = NetworkTableInstance.getDefault()
        .getStructTopic("AdvantageScope/SimSwervePose", Pose2d.struct).publish();
    private final StructArrayPublisher<SwerveModuleState> simSwerveModuleState = NetworkTableInstance.getDefault()
        .getStructArrayTopic("AdvantageScope/SimSwerveModuleState", SwerveModuleState.struct).publish();

    public SwerveSim() {
        super("SwerveSim", false);
    }
    public void drive(double xSpeed, double ySpeed, double rSpeed) {
        this.simPose = this.simPose.exp(
            new Twist2d(xSpeed * .02, ySpeed * .02, rSpeed * .02));
        SwerveModuleState[] states = SwerveConstants.KINEMATICS.toSwerveModuleStates(
            ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rSpeed, this.simPose.getRotation()));

        this.lastDesiredState = states;
    }

    @Override
    public void periodic() {
        this.simSwervePose.accept(this.simPose);
        this.simSwerveModuleState.accept(this.lastDesiredState);
    }

    @Override
    public void putDashboard() {}
}
