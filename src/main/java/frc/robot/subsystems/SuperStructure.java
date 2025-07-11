package frc.robot.subsystems;

import java.util.List;
import java.util.function.Supplier;
import java.util.stream.Stream;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.ZeroArmCmd;
import frc.robot.commands.ZeroElevatorCmd;
import frc.robot.commands.ZeroIntakeCmd;

public class SuperStructure extends SubsystemBase {
    private static SuperStructure SUPERSTRUCTURE;
    // ---------- Object ---------- //
    public StructureInput input = new StructureInput();
    private final Timer stateTime = new Timer();

    public SuperStructure() {
        SUPERSTRUCTURE = this;
    }

    public static SuperStructure getInstance() {
        return SUPERSTRUCTURE;
    }

    // ---------- State ---------- //
    public State state = State.Start;
    public enum State {
        Start(
            Elevator.State.Down,
            Arm.LifterState.Up, Arm.RollerState.idle,
            Intake.LifterState.Up, Intake.RollerState.Off),
        Rest(
            Elevator.State.PreHandoff,
            Arm.LifterState.Down, Arm.RollerState.idle),
        PrePopciclePickup(
            Elevator.State.PreHandoff,
            Arm.LifterState.PopciclePickup, Arm.RollerState.in,
            Intake.LifterState.Down, Intake.RollerState.Off),
        PopciclePickup(
            Elevator.State.PopciclePickup,
            Arm.LifterState.PopciclePickup, Arm.RollerState.in,
            Intake.LifterState.Up, Intake.RollerState.Off),
        ArmSourceIntake(
            Elevator.State.Down,
            Arm.LifterState.Up, Arm.RollerState.in),
        SourceIntake(
            Elevator.State.SourceIntake,
            Arm.LifterState.Down, Arm.RollerState.idle,
            Intake.LifterState.Up, Intake.RollerState.In),
        PreHandoff(
            Elevator.State.Handoff,
            Arm.LifterState.Down, Arm.RollerState.in,
            Intake.LifterState.Up, Intake.RollerState.SlowIn),
        Handoff(
            Elevator.State.Handoff,
            Arm.LifterState.Down, Arm.RollerState.in,
            Intake.LifterState.Up, Intake.RollerState.Out),
        PreScore(
            Elevator.State.PreScore,
            Arm.LifterState.Up, Arm.RollerState.idle,
            Intake.LifterState.Up, Intake.RollerState.Off),
        ReverseHandOff(
            Elevator.State.PreHandoff,
            Arm.LifterState.Down, Arm.RollerState.out,
            Intake.LifterState.Up, Intake.RollerState.In),
        PreThrough(
            Elevator.State.Through,
            Arm.LifterState.Down, Arm.RollerState.idle,
            Intake.LifterState.Through, Intake.RollerState.Off),
        Through(
            Elevator.State.Through,
            Arm.LifterState.Down, Arm.RollerState.idle,
            Intake.LifterState.Through, Intake.RollerState.TroughOut),
        
        // L2 Score state
        PrepareL2(
            Elevator.State.L2,
            Arm.LifterState.AboveScoreCoral, Arm.RollerState.idle,
            Intake.LifterState.Down, Intake.RollerState.Off),
        StartL2(
            Elevator.State.L2,
            Arm.LifterState.ScoreCoral, Arm.RollerState.idle,
            Intake.LifterState.Down, Intake.RollerState.Off),
        PlaceL2(
            Elevator.State.ScoreL2,
            Arm.LifterState.FinishScoreCoral, Arm.RollerState.idle,
            Intake.LifterState.Down, Intake.RollerState.Off),
        AfterL2(
            Elevator.State.PostL2,
            Arm.LifterState.Up, Arm.RollerState.slowout),
        
        // L3 Score state
        PrepareL3(
            Elevator.State.L3,
            Arm.LifterState.AboveScoreCoral, Arm.RollerState.idle),
        StartL3(
            Elevator.State.L3,
            Arm.LifterState.ScoreCoral, Arm.RollerState.idle),
        PlaceL3(
            Elevator.State.ScoreL3,
            Arm.LifterState.FinishScoreCoral, Arm.RollerState.off),
        AfterL3(
            Elevator.State.PostL3,
            Arm.LifterState.Up, Arm.RollerState.slowout),
        
        // L4 Score state
        PrepareL4(
            Elevator.State.L4,
            Arm.LifterState.AboveScoreCoral, Arm.RollerState.idle),
        StartL4(
            Elevator.State.L4,
            Arm.LifterState.ScoreL4Coral, Arm.RollerState.idle),
        PlaceL4(
            Elevator.State.ScoreL4,
            Arm.LifterState.FinishScoreL4Coral, Arm.RollerState.off),
        AfterL4(
            Elevator.State.PreHandoff,
            Arm.LifterState.Down, Arm.RollerState.slowout),
            
        // Aglae
        PreGetAlgae(
            Elevator.State.HighAglae,
            Arm.LifterState.SafeInsideRobotAngle, Arm.RollerState.in),
        GetAlgae(
            Elevator.State.AutoAlgae,
            Arm.LifterState.GetAlgae, Arm.RollerState.in),
        PostGetAlgae(
            Elevator.State.AutoAlgae,
            Arm.LifterState.PostAlgae, Arm.RollerState.algeaIdle),
        AlgaeRest(
            Elevator.State.AlgaeRest,
            Arm.LifterState.AlgaeUp, Arm.RollerState.algeaIdle),
        PreBarge(
            Elevator.State.Barge,
            Arm.LifterState.PreBarge, Arm.RollerState.algeaIdle),
        ScoreBarge(
            Elevator.State.Barge,
            Arm.LifterState.BargeScore, Arm.RollerState.out),
        AlgaeDescore(
            Elevator.State.AutoAlgae,
            Arm.LifterState.DescoreAlgae, Arm.RollerState.descore),
        AlgaeExit(
            Elevator.State.PreHandoff,
            Arm.LifterState.Down, Arm.RollerState.out),
        PreProcessor(
            Elevator.State.Processor,
            Arm.LifterState.Processor, Arm.RollerState.algeaIdle,
            Intake.LifterState.Down, Intake.RollerState.Off),
        ScoreProcessor(
            Elevator.State.Processor,
            Arm.LifterState.Processor, Arm.RollerState.slowout,
            Intake.LifterState.Down, Intake.RollerState.Off),
        PreAlgaeGroundIntake(
            State.Rest.elevator,
            Arm.LifterState.AlgaeGroundPickup, Arm.RollerState.off,
            Intake.LifterState.Down, Intake.RollerState.Off),
        AlgaeGroundIntake(
            Elevator.State.GroundAlgaeIntake,
            Arm.LifterState.AlgaeGroundPickup, Arm.RollerState.in,
            Intake.LifterState.Down, Intake.RollerState.Off),
        ExitAlgaeGroundIntake(
            Elevator.State.PreHandoff,
            Arm.LifterState.ExitAlgaeGroundPickup, Arm.RollerState.algeaIdle,
            Intake.LifterState.Down, Intake.RollerState.Off);

        public final Elevator.State elevator;
        public final Arm.LifterState armLifter;
        public final Arm.RollerState armRoller;
        public final Intake.LifterState intakeLifter;
        public final Intake.RollerState intakeRoller;

        State(
            Elevator.State elevator,
            Arm.LifterState armLifter, Arm.RollerState armRoller
        ) {
            this.elevator = elevator;
            this.armLifter = armLifter;
            this.armRoller = armRoller;
            this.intakeLifter = Intake.LifterState.OperatorControl;
            this.intakeRoller = Intake.RollerState.OperatorControl;
        }

        State(
            Elevator.State elevator,
            Arm.LifterState armLifter, Arm.RollerState armRoller,
            Intake.LifterState intakeLifter, Intake.RollerState intakeRoller
        ) {
            this.elevator = elevator;
            this.armLifter = armLifter;
            this.armRoller = armRoller;
            this.intakeLifter = intakeLifter;
            this.intakeRoller = intakeRoller;
        }
    }

    public enum ScoreLevel {
        Through(0),
        L2(1),
        L3(2),
        L4(3);

        public final int index;

        ScoreLevel(int index) {
            this.index = index;
        }
    }

    // ---------- Input ---------- //
    public static class StructureInput {
        public boolean wantExtend = false;
        public boolean wantGroundIntake = false;
        public boolean wantArmSourceIntake = false;
        public boolean wantSourceIntake = false;
        public boolean wantScore = false;
        public ScoreLevel wantedScoringLevel = ScoreLevel.L4;
        public boolean wantGetAlgae = false;
        public boolean wantDescoreAlgae = false;
        public boolean wantVerticalPickup = false;
        public boolean wantResetSuperstructure = false;
        public boolean wantScoreProcessor = false;
        public boolean wantAlgaeGroundIntake = false;
        public boolean wantPopsiclePickup = false;
    }

    // ---------- Transition ---------- //
    private final List<Transition> transitions = Stream.concat(
        Stream.of(
            // this just makes it so the elevator doesn't move until the operator wants to intake
            new Transition(State.Start, State.Rest, () -> this.input.wantGroundIntake || this.input.wantArmSourceIntake),
            // this is just for auto, probably should be done differently
            new Transition(State.Start, State.PreScore, () -> RobotState.isAutonomous()),

            // Arm source intaking, must be here
            new Transition(State.Rest, State.ArmSourceIntake, () -> this.input.wantArmSourceIntake),
            new Transition(State.ArmSourceIntake, State.Rest, () -> !this.input.wantArmSourceIntake || Arm.hasObject),

            // Intake source intaking
            new Transition(State.Rest, State.SourceIntake, () -> this.input.wantSourceIntake),
            new Transition(State.SourceIntake, State.Rest, () -> !this.input.wantSourceIntake || Intake.hasCoral),

            // Trough reverse handoff, must be here
            new Transition(State.PreScore, State.Rest, () -> this.input.wantedScoringLevel == ScoreLevel.Through || !Arm.hasObject),
            new Transition(State.Rest, State.ReverseHandOff, () -> (Arm.getInstance().atSetpoint() && Elevator.getInstance().atSetpoint() &&
                                                                this.input.wantedScoringLevel == ScoreLevel.Through && Arm.hasObject && !Intake.hasCoral &&
                                                                Intake.getInstance().getEffectiveLifterState() == Intake.LifterState.Up && Intake.getInstance().atSetpoint())),

            new Transition(State.ReverseHandOff, State.Rest, () -> Intake.hasCoral || this.input.wantResetSuperstructure),

            new Transition(State.Rest, State.PreThrough, () -> this.input.wantExtend && this.input.wantedScoringLevel == ScoreLevel.Through &&
                                                            Elevator.getInstance().atSetpoint() && Arm.getInstance().atSetpoint()),
            new Transition(State.PreThrough, State.Through, () -> Intake.getInstance().atSetpoint() && this.input.wantScore),
            new Transition(State.PreThrough, State.Rest, () -> !this.input.wantExtend),
            new Transition(State.Through, State.Rest, () -> !this.input.wantScore),

            new Transition(State.Rest, State.PreHandoff, () -> Elevator.getInstance().atSetpoint() && Arm.getInstance().atSafeReefDistance() &&
                                                            this.input.wantedScoringLevel != ScoreLevel.Through && Intake.hasCoral),
        
            new Transition(State.PreHandoff, State.Handoff, () -> Elevator.getInstance().atSetpoint() && Arm.getInstance().atSetpoint() && Intake.getInstance().atSetpoint()),
            new Transition(State.Handoff, State.Rest, () -> Arm.hasObject),
            new Transition(State.Rest, State.PreScore, () -> Arm.hasObject && this.input.wantedScoringLevel != ScoreLevel.Through),
            new Transition(State.Handoff, State.Rest, () -> this.input.wantResetSuperstructure),

            new Transition(State.PreScore, State.PrepareL4, () -> this.input.wantExtend && input.wantedScoringLevel == ScoreLevel.L4),
            new Transition(State.PreScore, State.PrepareL3, () -> this.input.wantExtend && input.wantedScoringLevel == ScoreLevel.L3),
            new Transition(State.PreScore, State.PrepareL2, () -> this.input.wantExtend && input.wantedScoringLevel == ScoreLevel.L2),

            // Algae Removal
            new Transition(State.AlgaeExit, State.PreGetAlgae, () -> this.input.wantGetAlgae && !Arm.hasObject),
            new Transition(State.Rest, State.PreGetAlgae, () -> this.input.wantGetAlgae && !Arm.hasObject),
            new Transition(State.PreGetAlgae, State.Rest, () -> !this.input.wantGetAlgae),

            new Transition(State.PreGetAlgae, State.GetAlgae, () -> Elevator.getInstance().atSetpoint()),
            new Transition(State.GetAlgae, State.PreGetAlgae, () -> !this.input.wantGetAlgae),

            new Transition(State.GetAlgae, State.PostGetAlgae, () -> Arm.hasObject),
            new Transition(State.PostGetAlgae, State.AlgaeRest, () -> Arm.getInstance().atSetpoint() && Arm.getInstance().atSafeReefDistance()),

            new Transition(State.AlgaeRest, State.AlgaeExit, () -> !Arm.hasObject),
            new Transition(State.AlgaeExit, State.Rest, () -> Arm.getInstance().atSetpoint() && Elevator.getInstance().atSetpoint()),

            new Transition(State.AlgaeRest, State.PreBarge, () -> this.input.wantExtend),
            new Transition(State.PreBarge, State.AlgaeRest, () -> !this.input.wantExtend),

            new Transition(State.PreBarge, State.ScoreBarge, () -> this.input.wantScore && Swerve.getInstance().atGoodScoringDistance()),
            new Transition(State.ScoreBarge, State.PreBarge, () -> (!this.input.wantExtend || !Arm.hasObject) && Arm.getInstance().atSafeBargeDistance()),

            new Transition(State.Rest, State.AlgaeDescore, () -> this.input.wantDescoreAlgae),
            new Transition(State.AlgaeDescore, State.Rest, () -> !this.input.wantDescoreAlgae),

            new Transition(State.AlgaeRest, State.PreProcessor, () -> this.input.wantScoreProcessor),
            new Transition(State.PreProcessor, State.ScoreProcessor, () -> this.input.wantScore),

            new Transition(State.ScoreProcessor, State.AlgaeRest, () -> !this.input.wantScoreProcessor && !Arm.hasObject && Arm.getInstance().atSafeProcessorDistance()),
            new Transition(State.PreProcessor, State.AlgaeRest, () -> !this.input.wantScoreProcessor && Arm.getInstance().atSafeProcessorDistance()),

            new Transition(State.Rest, State.PreAlgaeGroundIntake, () -> this.input.wantAlgaeGroundIntake && !Arm.hasObject),
            new Transition(State.PreAlgaeGroundIntake, State.AlgaeGroundIntake, () -> this.input.wantAlgaeGroundIntake && Intake.getInstance().atSetpoint()),
            new Transition(State.AlgaeGroundIntake, State.ExitAlgaeGroundIntake, () -> !this.input.wantAlgaeGroundIntake || Arm.hasObject),
            new Transition(State.ExitAlgaeGroundIntake, State.AlgaeRest, () -> Elevator.getInstance().atSetpoint()  && Arm.getInstance().atSetpoint()),

            new Transition(State.PopciclePickup, State.PrePopciclePickup, () -> !this.input.wantPopsiclePickup ||
            (RobotState.isAutonomous() && this.stateTime.hasElapsed(0.5))),
            new Transition(State.PrePopciclePickup, State.Rest, () -> !this.input.wantPopsiclePickup),
        
            new Transition(State.Rest, State.PrePopciclePickup, () -> this.input.wantPopsiclePickup && !Arm.hasObject),
            new Transition(State.PrePopciclePickup, State.PopciclePickup, () -> this.input.wantPopsiclePickup && Intake.getInstance().atSetpoint()),
            new Transition(State.PopciclePickup, State.PreScore, () -> this.stateTime.hasElapsed(0.5) && Arm.hasObject)),
        Stream.of(
            // scoreing Transitions
            this.scoringTransitions(ScoreLevel.L4, State.PrepareL4, State.StartL4, State.PlaceL4, State.AfterL4),
            this.scoringTransitions(ScoreLevel.L3, State.PrepareL3, State.StartL3, State.PlaceL3, State.AfterL3),
            this.scoringTransitions(ScoreLevel.L2, State.PrepareL2, State.StartL2, State.PlaceL2, State.AfterL2)
        ).flatMap(List::stream)
    ).toList();

    public List<Transition> scoringTransitions(ScoreLevel scoreLevel, State prepare, State start, State place, State after) {
        return List.<Transition>of(
            // Exit transitions (have to be first)
            new Transition(prepare, State.PreScore, () -> Arm.getInstance().atSetpoint() && (!Arm.hasObject || !this.input.wantExtend || this.input.wantedScoringLevel != scoreLevel)),
            new Transition(start, prepare, () -> !this.input.wantExtend || this.input.wantedScoringLevel != scoreLevel || !Arm.hasObject),
            // Normal transitions
            new Transition(prepare, start, () -> Elevator.getInstance().lazierAtSetpoint() && Arm.hasObject && this.input.wantExtend && this.input.wantedScoringLevel == scoreLevel),
            new Transition(start, place, () -> {}, () -> Elevator.getInstance().atSetpoint() && Arm.getInstance().atSetpoint() && this.input.wantScore),
            new Transition(place, after, () -> Elevator.getInstance().atSetpoint() && Arm.getInstance().atSetpoint() && Arm.getInstance().atSafePlacementDistance()),
            new Transition(after, State.Rest, () -> Arm.getInstance().isInsideFrame()));
    }

    public class Transition {
        public State currentState;
        public State nextState;
        public Runnable enterFunction;
        public Supplier<Boolean> booleanSupplier;

        public Transition(State cur, State next, Runnable enterFunction, Supplier<Boolean> booleanSupplier) {
            this.currentState = cur;
            this.nextState = next;
            this.enterFunction = enterFunction != null ? enterFunction : () -> {};
            this.booleanSupplier = booleanSupplier;
        }

        public Transition(State cur, State next, Supplier<Boolean> booleanSupplier) {
            this(cur, next, () -> {}, booleanSupplier);
        }
    }

    // ---------- Method ---------- //
    public void setStates() {
        Arm.setState(state.armLifter, state.armRoller);
        Elevator.state = state.elevator;
        Intake.setState(state.intakeLifter, state.intakeRoller);
    }

    public void emptyInputs() {
        this.input = new StructureInput();
    }

    @Override
    public void periodic() {
        if (!Elevator.isZeroed || !Arm.isZeroed) return;
        this.stateTime.start();

        for (Transition translate : this.transitions) {
            if (translate.currentState == state && translate.booleanSupplier.get()) {
                state = translate.nextState;
                this.stateTime.restart();
                translate.enterFunction.run();
                this.setStates();
                return;
            }
        }
    }

    // ---------- Function ---------- //
    public ParallelCommandGroup makeZeroAllSubsystemsCommand() {
        return new ParallelCommandGroup(
            new ZeroIntakeCmd(),
            new ZeroElevatorCmd(),
            new ZeroArmCmd());
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addStringProperty("State", () -> state.toString(), null);
        builder.addBooleanProperty("WantExtend", () -> this.input.wantExtend, null);
        builder.addBooleanProperty("WantScore", () -> this.input.wantScore, null);
        builder.addStringProperty("ScoreLevel", () -> this.input.wantedScoringLevel.toString(), null);
    }
}
