package frc.robot.subsystems;

import java.util.List;
import java.util.function.Supplier;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.SuperStructure.StructureInput;

public class SuperStructure extends SubsystemBase {
    private StructureInput input;

    public SuperStructure(SuperStructure.StructureInput input) {
        this.input = input;
    }
    
    // ---------- State ---------- //
    private State state = State.Start;
    public enum State {
        Start(
            Elevator.State.Down,
            Arm.LifterState.Up, Arm.RollerState.idle),
        Rest(
            Elevator.State.PreHandoff,
            Arm.LifterState.Down, Arm.RollerState.idle),
        PrePopciclePickup(
            Elevator.State.PreHandoff,
            Arm.LifterState.PrePopciclePickup, Arm.RollerState.in),
        PopciclePickup(
            Elevator.State.PopcicleHandoff,
            Arm.LifterState.PopciclePickup, Arm.RollerState.in),
        PreHandoff(
            Elevator.State.PreHandoff,
            Arm.LifterState.Down, Arm.RollerState.in),
        Handoff(
            Elevator.State.Handoff,
            Arm.LifterState.Down, Arm.RollerState.in),
        PreScore(
            Elevator.State.PreScore,
            Arm.LifterState.Up, Arm.RollerState.idle),
        ;

        public final Elevator.State elevator;
        public final Arm.LifterState armLifter;
        public final Arm.RollerState armRoller;

        State(Elevator.State elevator, Arm.LifterState armLifter, Arm.RollerState armRoller) {
            this.elevator = elevator;
            this.armLifter = armLifter;
            this.armRoller = armRoller;
        }
    }

    public enum ScoreLevel {
        Through,
        L2,
        L3,
        L4;
    }

    // ---------- Input ---------- /

// 不可變資料類
public record SuperstructureInputs(
    boolean wantExtend,
    boolean wantGroundIntake,
    boolean wantArmSourceIntake,
    boolean wantSourceIntake,
    boolean wantScore,
    ScoreLevel wantedScoringLevel,
    boolean wantGetAlgae,
    boolean wantDescoreAlgae,
    boolean wantResetSuperstructure,
    boolean wantScoreProcessor,
    boolean wantAlgaeGroundIntake
) {
    public static SuperstructureInputs empty() {
        return new SuperstructureInputs(
            false, false, false, false, false,
            ScoreLevel.L4, false, false, false,
            false, false
        );
    }
}

    // ---------- Transition ---------- //
    private final List<Transition> transitions = List.of(
        new Transition(State.Start, State.PreHandoff, () -> this.input.wantScore),
        new Transition(State.PreHandoff, State.Handoff, () -> this.input.wantScore)
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
    public void setState() {
        Arm.setState(state.armLifter, state.armRoller);
        Elevator.state = state.elevator;
    }

    @Override
    public void periodic() {
        // if (!Elevator.isZeroed() || !Arm.isZeroed()) return;
        System.out.println(input.wantExtend);
        // for (Transition translate : this.transitions) {
        //     if (translate.currentState == state && translate.booleanSupplier.get()) {
        //         state = translate.nextState;
        //         translate.enterFunction.run();
        //     }
        // }
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addStringProperty("State", () -> state.toString(), null);
        builder.addBooleanProperty("WantExtend", () -> this.input.wantExtend, null);
        builder.addBooleanProperty("WantScore", () -> this.input.wantScore, null);
        builder.addStringProperty("ScoreLevel", () -> this.input.wantedScoringLevel.toString(), null);
    }
}
