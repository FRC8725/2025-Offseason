package frc.robot;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.Set;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.simulation.AprilTagsSim;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.SuperStructure.State;

public class CoralSim extends SubsystemBase {
    private Pose3d pose = new Pose3d(-1000.0, -1000.0, -1000.0, new Rotation3d());
    private Pose3d[] scorePoses = new Pose3d[]{};
    private final Set<Pose3d> scoreLocations = new HashSet<>();
    private CoralSimLocation location = CoralSimLocation.Floor;
    private final Supplier<Pose2d> swervePose;
    private final SuperStructure superStructure;
    private SuperStructure.State lastState = SuperStructure.State.Start;
    private final ArrayList<Pose3d> coralSimLocation = Constants.getCoralSimualtionScoreLocation();
    @SuppressWarnings("unused")
    private final AprilTagsSim aprilTagsSim = new AprilTagsSim();

    private final StructPublisher<Pose3d> coralPosePublisher = NetworkTableInstance.getDefault()
        .getStructTopic("Coral/coral", Pose3d.struct).publish();
    private final StructArrayPublisher<Pose3d> reefPosesPublisher = NetworkTableInstance.getDefault()
        .getStructArrayTopic("Coral/Reef", Pose3d.struct).publish();

    public enum CoralSimLocation {
        Intake, Claw, Floor, Hiden
    }

    public enum CoralSimScoreLocation {
        G_L4(new Pose3d(5.27, 3.86, 1.75, new Rotation3d(0.0, Math.PI / 2.0, 0.0))),
        D_L4(new Pose3d(4.24, 3.27, 1.75, new Rotation3d(0.0, Math.PI / 2.0, 0.0))),
        C_L4(new Pose3d(3.96, 3.43, 1.75, new Rotation3d(0.0, Math.PI / 2.0, 0.0)));

        public final Pose3d pose;

        CoralSimScoreLocation(Pose3d pose) {
            this.pose = pose;
        }
    }

    public CoralSim(SuperStructure superStructure, Supplier<Pose2d> swervePose) {
        this.superStructure = superStructure;
        this.swervePose = swervePose;
    }

    public void simulationUpdate() {
        // coral to intake
        if (this.location == CoralSimLocation.Floor && this.canIntakeCoral() &&
            Intake.getInstance().getEffectiveLifterState() == Intake.LifterState.Down && Intake.getInstance().atSetpoint()) {
            this.setLocation(CoralSimLocation.Intake);
            Intake.hasCoral = true;
        }
        // PreHandoff to Handoff
        if (this.lastState != this.superStructure.state && this.superStructure.state == State.Handoff) {
            this.setLocation(CoralSimLocation.Claw);
            Arm.hasObject = true;
            Intake.hasCoral = false;
        }
        // Score
        if (this.lastState != this.superStructure.state && (this.superStructure.state == State.PlaceL4 ||
            this.superStructure.state == State.PlaceL3 || this.superStructure.state == State.PlaceL2))
        {
            this.setLocation(CoralSimLocation.Hiden);
            int index = this.superStructure.input.get().wantedScoringLevel.index;
            this.addScoringPose(this.coralSimLocation.get(index * 4));
            Arm.hasObject = false;
        }
        this.lastState = this.superStructure.state;
        switch (this.location) {
            case Claw:
                Pose3d armBasePose = new Pose3d(this.swervePose.get())
                    .plus(new Transform3d(
                        0.027525, 0.0, 0.3003578 + Elevator.getInstance().getHeight(),
                        new Rotation3d(Arm.getInstance().getPosition() - Math.PI, 0.0, 0.0)));
                Transform3d armCoralOffset = new Transform3d(
                    0.1337, 0.0, 0.598131,
                    new Rotation3d(0.0, 0.0, Math.PI / 2.0));
                this.pose = armBasePose.plus(armCoralOffset);
                break;
        
            case Intake:
                Pose3d intakeBasePose = new Pose3d(this.swervePose.get())
                    .plus(new Transform3d(
                        0.28575, 0.0, 0.1905028,
                        new Rotation3d(0.0, Intake.getInstance().getPosition(), 0.0)));
                Transform3d coralOffset = new Transform3d(
                    -0.120161, 0.0, 0.213,
                    new Rotation3d(0.0, 0.0, Math.PI / 2.0));
                this.pose = intakeBasePose.plus(coralOffset);
                break;
            
            case Floor:
                this.pose = this.canIntakeCoral() ? this.pose : new Pose3d(0.64, 0.1, Units.inchesToMeters(4.5 / 2.0), new Rotation3d(0.0, 0.0, Math.PI / 2.0));
                break;

            default:
                this.pose = new Pose3d(-1000.0, -1000.0, -500.0, new Rotation3d());
        }

        this.scorePoses = this.scoreLocations.stream()
            .toArray(Pose3d[]::new);

        this.coralPosePublisher.accept(this.pose);
        this.reefPosesPublisher.accept(
            this.scoreLocations.stream().toArray(Pose3d[]::new));
    }

    public void setLocation(CoralSimLocation location) {
        this.location = location;
    }

    public void addScoringPose(Pose3d pose) {
        this.scoreLocations.add(pose);
    }

    public void removeScoringPose(Pose3d pose) {
        this.scoreLocations.remove(pose);
    }

    public void scoreCoral(State placeState) {

    }

    public boolean canIntakeCoral() {
        double swerveAngle = this.swervePose.get().getRotation().getDegrees();
        double coralAngle = this.pose.toPose2d().getRotation().getDegrees() - 90.0;
        boolean degree = Math.abs(swerveAngle - coralAngle) < 20.0;

        Translation2d swervePose = this.swervePose.get().getTranslation();
        Translation2d coralPose = this.pose.getTranslation().toTranslation2d();
        double distance = swervePose.getDistance(coralPose);

        return degree && (distance < 0.68);
    }
}
