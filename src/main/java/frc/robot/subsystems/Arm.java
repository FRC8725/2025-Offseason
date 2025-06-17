package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
    private final TalonFX roller = new TalonFX(15);
    private final TalonFX lifter = new TalonFX(16);
    private final StatusSignal<Current> statorCurrent = this.roller.getStatorCurrent();
    public static boolean hasObject = false;
    public static boolean isZeroed = false;

    // ----------- Debouncer ----------- //
    private final Debouncer coralCurrentDebouncer = new Debouncer(0.15, DebounceType.kBoth);
    private final Debouncer algaeCurrentDebouncer = new Debouncer(0.25, DebounceType.kBoth);

    // ----------- State ----------- //
    private static RollerState rollerState = RollerState.off;
    public enum RollerState {
        off(0.0),
        idle(-0.035),
        algeaIdle(-0.225),
        in(-1.0),
        out(1.0),
        slowout(0.075);

        public final double value;

        RollerState(double value) {
            this.value = value;
        }
    }

    private static LifterState lifterState = LifterState.Down;
    public enum LifterState {
        Down(0.0), 
        Up(0.0),
        PrePopciclePickup(0.0),
        PopciclePickup(0.0),
        AboveScoreCoral(0.0),
        ScoreCoral(0.0),
        ScoreL4Coral(0.0),
        FinishScoreL4Coral(0.0),
        FinishScoreCoral(0.0);

        public final double value;

        LifterState(double value) {
            this.value = value;
        }
    }

    public Arm(Supplier<Pose3d> carriagePose) {
        this.configRollerMotor(false);
        this.statorCurrent.setUpdateFrequency(100.0);
        rollerState = RollerState.in;

        this.carriagePose = carriagePose;
    }

    // ----------- Config ----------- //
    public void configRollerMotor(boolean reverse) {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput
            .withNeutralMode(NeutralModeValue.Brake)
            .withInverted(reverse ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive);
        config.CurrentLimits
            .withStatorCurrentLimitEnable(true)
            .withStatorCurrentLimit(80.0);
        this.roller.getConfigurator().apply(config);
    }

    // ---------- Method ---------- //
    public void resetRelativeFromAbsolute() {
        isZeroed = true;
    }

    public static void setState(LifterState lifter, RollerState roller) {
        lifterState = lifter;
        rollerState = roller;
    }

    @Override
    public void periodic() {
        this.statorCurrent.refresh();
        boolean undebouncedHasObject = this.statorCurrent.getValueAsDouble() >
            (rollerState == RollerState.idle ?
                Constants.Arm.IDEL_CURRENT_DRAW : Constants.Arm.CURRENT_DRAW);

        boolean debouncedHasCoral = this.coralCurrentDebouncer.calculate(undebouncedHasObject);
        boolean debouncedHasAlgae = this.algaeCurrentDebouncer.calculate(undebouncedHasObject);
        hasObject = debouncedHasAlgae;
        // this.hasObject = (this.rollerState == RollerState.algeaIdle) ? debouncedHasAlgae : debouncedHasCoral;
        if (hasObject) rollerState = RollerState.idle;
        this.roller.set(rollerState.value);
    }

    // ---------- Function ---------- //
    public double getPosition() {
        return Units.rotationsToRadians(this.lifter.getPosition().getValueAsDouble());
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addBooleanProperty("HasObject", () -> hasObject, null);
        builder.addStringProperty("RollerState", () -> rollerState.toString(), null);
        builder.addDoubleProperty("RollerCurrent", () -> this.statorCurrent.getValueAsDouble(), null);
    }

    // ---------- Simulation ---------- //
    private final StructPublisher<Pose3d> armComponent = NetworkTableInstance.getDefault()
        .getStructTopic("Component/Arm",  Pose3d.struct).publish();
    private final Supplier<Pose3d> carriagePose;
    private final SingleJointedArmSim armSim = new SingleJointedArmSim(
        DCMotor.getFalcon500(1),
        224.0 / 3.0,
        7.0,
        0.65,
        -Math.PI,
        Math.PI,
        false,
        0.0);
    
    public Pose3d getArmComponentPose() {
        return this.carriagePose.get()
            .plus(new Transform3d(0.0, 0.0, 0.300355,
                new Rotation3d(this.getPosition(), 0.0, 0.0)));
    }

    public void simulationUpdate() {
        this.armSim.setInput(this.lifter.get());
        this.armSim.update(0.020);
        // this.lifter.setPosition(Math.IEEEremainder(2.0 * Math.PI / 4.0 * Timer.getTimestamp(), 2.0 * Math.PI));
        this.lifter.setPosition(Units.radiansToRotations(this.armSim.getAngleRads()));

        this.armComponent.accept(this.getArmComponentPose());
    }
}
