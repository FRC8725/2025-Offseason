package frc.robot.commands;

import java.text.Collator;
import java.util.Comparator;
import java.util.List;
import java.util.function.Supplier;
import java.util.stream.Collectors;

import choreo.trajectory.EventMarker;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.Swerve;

public class AutoRunnerCmd extends Command {
	private SuperStructure superStructure;
	private Trajectory<SwerveSample> trajectory;
	private final Swerve swerve;

	public AutoRunnerCmd(SuperStructure superStructure, Trajectory<SwerveSample> trajectory, Swerve swerve) {
		this.superStructure = superStructure;
		this.trajectory = trajectory;
		this.swerve = swerve;
		this.addRequirements(this.superStructure);
	}

	private final List<Event> events = this.trajectory.events().stream()
		.sorted(Comparator.comparingDouble(e -> e.timestamp))
		.map(this::eventFromEventMarker)
		.collect(Collectors.toList());

	public class Event {
		public final String name;
		public final double timestamp;
		public final SuperStructure.StructureInput inputs;
		public final Supplier<Boolean> waitCondition;
		public final boolean requireAlignment;

		public Event copyWithTimestamp(double newTimestamp) {
			return new Event(this.name, newTimestamp, this.inputs, this.waitCondition, this.requireAlignment);
		}

		public Event(String name, SuperStructure.StructureInput inputs) {
            this(name, 0.0, inputs, null, false);
        }

        public Event(String name, SuperStructure.StructureInput inputs, Supplier<Boolean> waitCondition, boolean requireAlignment) {
            this(name, 0.0, inputs, waitCondition, requireAlignment);
    	}

        public Event(String name, double timestamp, SuperStructure.StructureInput inputs, Supplier<Boolean> waitCondition, boolean requireAlignment) {
            this.name = name;
            this.timestamp = timestamp;
            this.inputs = inputs;
            this.waitCondition = waitCondition;
            this.requireAlignment = requireAlignment;
    	}
	}

	private final List<Event> eventTypes = List.of(
		new Event(
			"startL4",
			new SuperStructure.StructureInput() {{
				wantedScoringLevel = SuperStructure.ScoreLevel.L4;
				wantExtend = true;
			}}),
		new Event(
			"L4",
			new SuperStructure.StructureInput() {{
				wantedScoringLevel = SuperStructure.ScoreLevel.L4;
				wantExtend = true;
				wantScore = true;
			}},
			() -> !Arm.hasObject || (this.superStructure.state == SuperStructure.State.PlaceL4),
			true),
		new Event(
			"intakeDown",
			new SuperStructure.StructureInput() {{
				wantGroundIntake = true;
			}}),
		new Event(
			"intakeDownNoHandoff",
			new SuperStructure.StructureInput() {{
				wantGroundIntake = true;
				wantedScoringLevel = SuperStructure.ScoreLevel.Through;
			}}),
		new Event(
			"intakeUp",
			new SuperStructure.StructureInput()),
		new Event(
			"zeroInputs",
			new SuperStructure.StructureInput()),
		new Event(
			"startGetAlgea",
			new SuperStructure.StructureInput() {{
				wantGetAlgae = true;
			}}),
		new Event(
			"waitGetAlgae",
			new SuperStructure.StructureInput() {{
				wantGetAlgae = true;
			}},
			() -> Arm.hasObject,
			true),
		new Event(
			"extendAlgae",
			new SuperStructure.StructureInput() {{
				wantExtend = true;
			}}),
		new Event(
			"scoreAlgae",
			new SuperStructure.StructureInput() {{
				wantExtend = true;
				wantScore = true;
			}}),
		new Event(
			"verticalCoral",
			new SuperStructure.StructureInput() {{
				wantPopsiclePickup = true;
			}}));

	private Event eventFromEventMarker(EventMarker eventMarker) {
		for (Event type : this.eventTypes) {
			if (type.requireAlignment) assert type.waitCondition != null;
			if (type.name == eventMarker.event) {
				return type.copyWithTimestamp(eventMarker.timestamp);
			}
		}
		throw new Error("Unrecognized event mark D:");
	}

	@Override
	public void initialize() {
		Pose2d initPose = this.trajectory.getInitialPose(Robot.isRedAlliance).get();
		if (this.swerve.getPose().getTranslation().getDistance(initPose.getTranslation()) > Constants.Swerve.STRATING_TOLERANCE) this.swerve.resetPose(initPose);
	}

	@Override
	public void execute() {
	}

	@Override
	public void end(boolean interrupted) {
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
