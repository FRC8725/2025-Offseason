package frc.robot;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.Map;
import java.util.Optional;
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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.simulation.AprilTagsSim;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.SuperStructure.ScoreLevel;
import frc.robot.subsystems.SuperStructure.State;

public class CoralSim extends SubsystemBase {
    private Pose3d pose = new Pose3d(-1000.0, -1000.0, -1000.0, new Rotation3d());
    private final Set<Pose3d> scoreLocations = new HashSet<>();
    private CoralSimLocation location = CoralSimLocation.Claw;
    private final Supplier<Pose2d> swervePose;
    private final SuperStructure superStructure;
    private SuperStructure.State lastState = SuperStructure.State.Start;
    private SuperStructure.ScoreLevel lastLevel = ScoreLevel.L4;
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
        if (this.lastState != this.superStructure.state && this.superStructure.state == State.Handoff && 
            this.superStructure.input.wantedScoringLevel != ScoreLevel.Through)
        {
            this.setLocation(CoralSimLocation.Claw);
            Arm.hasObject = true;
            Intake.hasCoral = false;
        }
        // Reverse hand off
        if (this.lastState != this.superStructure.state && this.superStructure.state == State.ReverseHandOff) {
            this.setLocation(CoralSimLocation.Intake);
            Arm.hasObject = false;
            Intake.hasCoral = true;
        }
        if (this.lastLevel != this.superStructure.input.wantedScoringLevel && this.superStructure.input.wantedScoringLevel != ScoreLevel.Through &&
            this.superStructure.state == State.Rest)
        {
            this.setLocation(CoralSimLocation.Claw);
            Arm.hasObject = true;
            Intake.hasCoral = false;    
        }
        // L2~L4 Score
        if (this.lastState != this.superStructure.state && (this.superStructure.state == State.PlaceL4 ||
            this.superStructure.state == State.PlaceL3 || this.superStructure.state == State.PlaceL2))
        {
            this.setLocation(CoralSimLocation.Hiden);
            Optional<Map.Entry<Integer, Pose2d>> closestFudged = Swerve.getInstance().getClosestFudgedScoringPose();
            if (!closestFudged.isPresent()) return;
            int index = closestFudged.get().getKey();
            this.addScoringPose(this.coralSimLocation.get(index * 4 + SuperStructure.getInstance().input.wantedScoringLevel.index));
            Arm.hasObject = false;
            this.setLocation(CoralSimLocation.Floor);
        }
        // Through
        if (this.lastState != this.superStructure.state && this.superStructure.state == State.Through) {
            this.setLocation(CoralSimLocation.Hiden);
            Optional<Map.Entry<Integer, Pose2d>> closestFudged = Swerve.getInstance().getClosestFudgedScoringPose();
            if (!closestFudged.isPresent()) return;
            int index = closestFudged.get().getKey();

            // this.addScoringPose(this.coralSimLocation.get(((index / 4) * 8) + (index % 4) * 4));
            Intake.hasCoral = false;
            this.setLocation(CoralSimLocation.Floor);
        }
        // PreGoundIntakeAlgae
        if (this.lastState != this.superStructure.state && this.superStructure.state == State.AlgaeGroundIntake) {
            this.setLocation(CoralSimLocation.Claw);
            Arm.hasObject = true;
        }
        // PopciclePickup
        if (this.lastState != this.superStructure.state && this.superStructure.state == State.PopciclePickup) {
            this.setLocation(CoralSimLocation.Claw);
            Arm.hasObject = true;
        }
        this.lastState = this.superStructure.state;
        this.lastLevel = this.superStructure.input.wantedScoringLevel;
        if (DriverStation.isAutonomous() && Arm.hasObject) {
            this.setLocation(CoralSimLocation.Claw);
        }
        switch (this.location) {
            case Claw:
                Pose3d armBasePose = new Pose3d(this.swervePose.get())
                    .plus(new Transform3d(
                        0.027525, 0.0, 0.3003578 + Elevator.getInstance().getHeight(),
                        new Rotation3d(-Arm.getInstance().getPosition() - Math.PI, 0.0, 0.0)));
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
                this.pose = new Pose3d(1.81, 0.8, Units.inchesToMeters(4.5 / 2.0), new Rotation3d(0.0, 0.0, Math.PI / 2.0 - Math.PI / 4.0 - Math.PI));
                // this.pose = new Pose3d(2.08, 7.38, Units.inchesToMeters(4.5 / 2.0), new Rotation3d(0.0, 0.0, Math.PI / 2.0));
                break;

            default:
                this.pose = new Pose3d(-1000.0, -1000.0, -500.0, new Rotation3d());
        }

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
        swerveAngle = swerveAngle > 180.0 ? swerveAngle - 180.0 : swerveAngle;
        coralAngle = coralAngle > 180.0 ? coralAngle - 180.0 : coralAngle;
        boolean degree = Math.abs(swerveAngle - coralAngle) < 20.0;

        Translation2d swervePose = this.swervePose.get().getTranslation();
        Translation2d coralPose = this.pose.getTranslation().toTranslation2d();
        double distance = swervePose.getDistance(coralPose);

        return degree && (distance < 0.68);
    }
}
