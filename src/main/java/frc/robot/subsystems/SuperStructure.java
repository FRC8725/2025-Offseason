package frc.robot.subsystems;

import java.util.List;
import java.util.function.Supplier;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SuperStructure extends SubsystemBase {
    // ---------- State ---------- //
    private State state = State.Start;
    public enum State {
        Start(Elevator.State.Down, Arm.LifterState.Up, Arm.RollerState.idle);

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

    // ---------- Input ---------- //
    private final StructureInput input = new StructureInput();
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
        new Transition(State.Start, State.Start, () -> false)
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
        if (!Elevator.isZeroed() || !Arm.isZeroed()) return;

        for (Transition translate : this.transitions) {
            if (translate.currentState == state && translate.booleanSupplier.get()) {
                state = translate.nextState;
                translate.enterFunction.run();
            }
        }
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addStringProperty("State", () -> state.toString(), null);
        builder.addBooleanProperty("WantExtend", () -> this.input.wantExtend, null);
        builder.addBooleanProperty("WantScore", () -> this.input.wantScore, null);
        builder.addStringProperty("ScoreLevel", () -> this.input.wantedScoringLevel.toString(), null);
    }
}
