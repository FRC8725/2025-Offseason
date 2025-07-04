package frc.robot.commands;

import java.util.Comparator;
import java.util.List;
import java.util.Objects;
import java.util.function.Supplier;
import java.util.stream.Collectors;

import choreo.trajectory.EventMarker;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.Swerve;

public class AutoRunnerCmd extends Command {
	private SuperStructure superStructure;
	private SuperStructure.StructureInput postAlignInputs = null;
	private Trajectory<SwerveSample> trajectory;
	private Event currentWaitEvent = null;
	private boolean waitingForAlign = false;
	private Pose2d lastPose = null;
	private int eventI = 0;
	private final Swerve swerve;
	private final Timer timer = new Timer();
	private final List<Event> events;

	public AutoRunnerCmd(SuperStructure superStructure, Trajectory<SwerveSample> trajectory, Swerve swerve) {
		this.superStructure = superStructure;
		this.trajectory = trajectory;
		this.swerve = swerve;
		this.addRequirements(this.superStructure);

		this.events = this.trajectory.events().stream()
			.sorted(Comparator.comparingDouble(e -> e.timestamp))
			.map(this::eventFromEventMarker)
			.collect(Collectors.toList());
	}

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
			if (type.name.equals(eventMarker.event)) {
				return type.copyWithTimestamp(eventMarker.timestamp);
			}
		}
		throw new Error("Unrecognized event mark D:");
	}

	private boolean shouldRunEvent(Event ev) {
		return this.timer.hasElapsed(ev.timestamp) || this.timer.hasElapsed(this.trajectory.getTotalTime());
	}

	@Override
	public void initialize() {
		this.timer.restart();
		Pose2d initPose = this.trajectory.getInitialPose(Robot.isRedAlliance).get();
		if (this.swerve.getPose().getTranslation().getDistance(initPose.getTranslation()) > Constants.Swerve.STRATING_TOLERANCE) this.swerve.resetPose(initPose);
	}

	@Override
	public void execute() {
		if (this.timer.isRunning()) assert this.currentWaitEvent == null;
		else assert this.currentWaitEvent != null;

		if (this.waitingForAlign) {
			assert this.lastPose != null;
			if (this.swerve.withinTolerance(Objects.requireNonNull(this.lastPose).getTranslation())) {
				this.superStructure.input = this.postAlignInputs;
				this.waitingForAlign = false;
			}
			this.swerve.followPose(Objects.requireNonNull(this.lastPose));
			return;
		}

		if (this.currentWaitEvent != null && this.currentWaitEvent.waitCondition != null && this.currentWaitEvent.waitCondition.get()) {
			this.superStructure.emptyInputs();
			this.currentWaitEvent = null;
		} else if (this.currentWaitEvent == null && this.eventI < events.size() && this.shouldRunEvent(this.events.get(this.eventI))) {
			Event ev = this.events.get(eventI++);
			if (ev.requireAlignment) {
				this.postAlignInputs = ev.inputs;
			} else {
				this.superStructure.input = ev.inputs;
			}

			SmartDashboard.putBoolean("ev", ev.inputs.wantGroundIntake);

			if (ev.waitCondition != null) {
				this.currentWaitEvent = ev;
				this.waitingForAlign = ev.requireAlignment;
				this.swerve.stopModules();
				this.timer.stop();
			}
		}

		if (this.currentWaitEvent == null) {
			this.timer.start();
			SwerveSample sample = this.trajectory.sampleAt(this.timer.get(), Robot.isRedAlliance)
				.orElse(this.trajectory.getFinalSample(Robot.isRedAlliance).get());
			this.lastPose = sample.getPose();
			this.swerve.followSample(sample);
		}
		SmartDashboard.putString("CurrentWaitEvent", this.currentWaitEvent == null ? "" : this.currentWaitEvent.name);
	}

	@Override
	public void end(boolean interrupted) {
		this.swerve.stopModules();
		this.superStructure.emptyInputs();
	}

	@Override
	public boolean isFinished() {
		return this.timer.hasElapsed(this.trajectory.getTotalTime()) &&
			this.currentWaitEvent == null &&
			this.eventI >= this.events.size();
	}

	@Override
	public void initSendable(SendableBuilder builder) {
		builder.addBooleanProperty("Is Timer running", () -> this.timer.isRunning(), null);
		builder.addDoubleProperty("Time", () -> this.timer.get(), null);
		builder.addDoubleProperty("Trajectory Total Time", () -> this.trajectory.getTotalTime(), null);
		builder.addStringProperty("Current wait event", () -> this.currentWaitEvent.name, null);
	}
}
