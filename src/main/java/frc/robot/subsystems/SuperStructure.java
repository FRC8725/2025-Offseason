package frc.robot.subsystems;

import java.util.List;
import java.util.function.Supplier;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SuperStructure extends SubsystemBase {
    public Supplier<StructureInput> input;

    public SuperStructure(Supplier<StructureInput> input) {
        this.input = input;
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
            Arm.LifterState.PrePopciclePickup, Arm.RollerState.in,
            Intake.LifterState.Down, Intake.RollerState.Off),
        PopciclePickup(
            Elevator.State.PopcicleHandoff,
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
            Elevator.State.PreHandoff,
            Arm.LifterState.Down, Arm.RollerState.in,
            Intake.LifterState.Up, Intake.RollerState.Off),
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
            Arm.LifterState.AboveScoreCoral, Arm.RollerState.idle),
        StartL2(
            Elevator.State.L2,
            Arm.LifterState.ScoreCoral, Arm.RollerState.idle),
        PlaceL2(
            Elevator.State.ScoreL2,
            Arm.LifterState.FinishScoreCoral, Arm.RollerState.slowout),
        AfterL2(
            Elevator.State.PostL2,
            Arm.LifterState.Up, Arm.RollerState.out),
        
        // L3 Score state
        PrepareL3(
            Elevator.State.L3,
            Arm.LifterState.AboveScoreCoral, Arm.RollerState.idle),
        StartL3(
            Elevator.State.L3,
            Arm.LifterState.ScoreCoral, Arm.RollerState.idle),
        PlaceL3(
            Elevator.State.ScoreL2,
            Arm.LifterState.FinishScoreCoral, Arm.RollerState.slowout),
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
            Arm.LifterState.Down, Arm.RollerState.slowout)
            

        ;

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
        Through,
        L2,
        L3,
        L4;
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
    private final List<Transition> transitions = List.of(
        // new Transition(State.Start, State.PreHandoff, () -> this.input.get().wantGroundIntake || this.input.get().wantArmSourceIntake),
        new Transition(State.Start, State.PreHandoff, () -> true),
        new Transition(State.Start, State.PreScore, () -> RobotState.isAutonomous()),

        new Transition(State.Rest, State.ArmSourceIntake, () -> this.input.get().wantArmSourceIntake),
        new Transition(State.ArmSourceIntake, State.Rest, () -> !this.input.get().wantArmSourceIntake || Arm.hasObject),

        new Transition(State.Rest, State.SourceIntake, () -> this.input.get().wantSourceIntake),
        new Transition(State.SourceIntake, State.Rest, () -> !this.input.get().wantSourceIntake || Intake.hasCoral),

        new Transition(State.PreScore, State.Rest, () -> this.input.get().wantedScoringLevel == ScoreLevel.Through || !Arm.hasObject),
        new Transition(State.Rest, State.ReverseHandOff, () -> this.input.get().wantedScoringLevel == ScoreLevel.Through && Arm.hasObject && !Intake.hasCoral),

        new Transition(State.ReverseHandOff, State.Rest, () -> this.input.get().wantedScoringLevel == ScoreLevel.Through || !Arm.hasObject),

        new Transition(State.Rest, State.PreThrough, () ->  this.input.get().wantExtend && this.input.get().wantedScoringLevel == ScoreLevel.Through),
        new Transition(State.PreThrough, State.Through, () -> this.input.get().wantScore),
        new Transition(State.PreThrough, State.Rest, () -> !this.input.get().wantExtend),
        new Transition(State.Through, State.Rest, () -> !this.input.get().wantScore)
    );

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

    @Override
    public void periodic() {
        // if (!Elevator.isZeroed() || !Arm.isZeroed) return;

        for (Transition translate : this.transitions) {
            if (translate.currentState == state && translate.booleanSupplier.get()) {
                state = translate.nextState;
                translate.enterFunction.run();
                this.setStates();
                return;
            }
        }
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addStringProperty("State", () -> state.toString(), null);
        builder.addBooleanProperty("WantExtend", () -> this.input.get().wantExtend, null);
        builder.addBooleanProperty("WantScore", () -> this.input.get().wantScore, null);
        builder.addStringProperty("ScoreLevel", () -> this.input.get().wantedScoringLevel.toString(), null);
    }
}
