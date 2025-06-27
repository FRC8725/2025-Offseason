package frc.robot.subsystems;

import java.util.Comparator;
import java.util.List;
import java.util.function.Supplier;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.lib.math.MathUtils;

public class Arm extends SubsystemBase {
    private static Arm ARM;
    // ---------- Object ---------- //
    private final TalonFX roller = new TalonFX(15);
    private final TalonFX lifter = new TalonFX(16);
    private final DutyCycleEncoder encoder = new DutyCycleEncoder(0);
    private final StatusSignal<Current> statorCurrent = this.roller.getStatorCurrent();
    private final Supplier<Boolean> wantOffsetArmPositive;
    private final Supplier<Boolean> wantOffsetArmNegative;
    public static boolean hasObject = false;
    public static boolean isZeroed = false;
    private double offsetRad = 0.0;
    private boolean isStuck = false;
    private double lastUpdatedTick = -1.0;
    private double lastCachedValue = 0.0;

    // ----------- Debouncer ----------- //
    private final Debouncer coralCurrentDebouncer = new Debouncer(0.15, DebounceType.kBoth);
    private final Debouncer algaeCurrentDebouncer = new Debouncer(0.25, DebounceType.kBoth);

    // ----------- State ----------- //
    public static RollerState rollerState = RollerState.off;
    public enum RollerState {
        off(0.0),
        idle(-0.035),
        algeaIdle(-0.135),
        fastIdle(-0.1),
        in(-1.0),
        out(1.0),
        slowout(0.075),
        descore(0.8);

        public final double value;

        RollerState(double value) {
            this.value = value;
        }
    }

    public static LifterState lifterState = LifterState.Up;
    public enum LifterState {
        Down(0.0, MirrorType.ActuallyFixedAngle), 
        Up(Math.PI, MirrorType.FixedAngle),
        PrePopciclePickup(0.0, MirrorType.AlgaeScore),
        PopciclePickup(0.0, MirrorType.ActuallyFixedAngle),
        AboveScoreCoral(Units.degreesToRadians(160.0), MirrorType.ClosestToReef),
        ScoreCoral(Units.degreesToRadians(130.0), MirrorType.ClosestToReef),
        ScoreL4Coral(0.0, MirrorType.ClosestToReef),
        FinishScoreL4Coral(0.0, MirrorType.ClosestToReef),
        FinishScoreCoral(0.0, MirrorType.ClosestToReef),
        SafeInsideRobotAngle(Math.PI - Constants.Arm.SAFE_INSIDE_ROBOT_ANGLE, MirrorType.ClosestToReef),
        GetAlgae(Units.degreesToRadians(100.0), MirrorType.ClosestToReef),
        PostAlgae(Units.degreesToRadians(110.0), MirrorType.ClosestToReef),
        DescoreAlgae(Units.degreesToRadians(110.0), MirrorType.ClosestToReef),
        AlgaeUp(Math.PI, MirrorType.FixedAngle),
        PreBarge(Units.degreesToRadians(160.0), MirrorType.AlgaeScore),
        BargeScore(Units.degreesToRadians(160.0), MirrorType.AlgaeScore),
        Processor(Units.degreesToRadians(70.0), MirrorType.ProcessorScore),
        AlgaeGroundPickup(Units.degreesToRadians(-78.0), MirrorType.ActuallyFixedAngle),
        ExitAlgaeGroundPickup(Units.degreesToRadians(-95.0), MirrorType.ActuallyFixedAngle);

        public final double value;
        public final MirrorType mirrorType;

        LifterState(double value, MirrorType mirrorType) {
            this.value = value;
            this.mirrorType = mirrorType;
        }

        public double desiredAngle(double position, Side sideCloserToReef, Side sideCloserToBarge, Side sideCloserToProcessor) {
            switch (this.mirrorType) {
                case ActuallyFixedAngle:
                    return this.value;
                case FixedAngle:
                    return this.value;
                case ClosestToPosition:
                    return this.value * (position > 0.0 ? 1.0 : -1.0);
                case ClosestToReef:
                    switch (sideCloserToReef) {
                        case Left:
                            return -this.value;
                        case Right:
                            return this.value;
                        default:
                            return Math.PI;
                    }
                case AlgaeScore:
                    switch (sideCloserToBarge) {
                        case Left:
                            return -this.value;
                        case Right:
                            return this.value;
                        default:
                            return Math.PI;
                    }
                case ProcessorScore:
                    switch (sideCloserToProcessor) {
                        case Left:
                            return -this.value;
                        case Right:
                            return this.value;
                        default:
                            return Math.PI;
                    }
                default:
                    return Math.PI;
            }
        }
    }

    public enum MirrorType {
        FixedAngle,
        ActuallyFixedAngle,
        ClosestToReef,
        ClosestToPosition,
        AlgaeScore,
        ProcessorScore
    }

    public enum Side {
        Left, Right, Neither
    }

    // ----------- Form ----------- //
    public static final InterpolatingDoubleTreeMap elevatorToArm = new InterpolatingDoubleTreeMap();
    public static final InterpolatingDoubleTreeMap elevatorToArmWhenIntakeDown = new InterpolatingDoubleTreeMap();

    public Arm(Supplier<Boolean> wantOffsetArmPositive, Supplier<Boolean> wantOffsetArmNegative) {
        ARM = this;
        this.configRollerMotor();
        this.resetRelativeFromAbsolute();
        this.statorCurrent.setUpdateFrequency(100.0);

        for (Pair<Double, Double> pair : Constants.armElevatorPairs) {
            elevatorToArm.put(pair.getSecond(), pair.getFirst());
        }
        for (Pair<Double, Double> pair : Constants.armInterpolationIntakeDown) {
            elevatorToArmWhenIntakeDown.put(pair.getSecond(), pair.getFirst());
        }

        this.wantOffsetArmPositive = wantOffsetArmPositive;
        this.wantOffsetArmNegative = wantOffsetArmNegative;
    }

    public static Arm getInstance() {
        return ARM;
    }

    // ----------- Config ----------- //
    public void configRollerMotor() {
        TalonFXConfiguration rollerConfig = new TalonFXConfiguration();
        rollerConfig.MotorOutput
            .withNeutralMode(NeutralModeValue.Brake)
            .withInverted(InvertedValue.CounterClockwise_Positive);
        rollerConfig.CurrentLimits
            .withStatorCurrentLimitEnable(true)
            .withStatorCurrentLimit(80.0);

        TalonFXConfiguration lifterConfig = new TalonFXConfiguration();
        Slot0Configs slot0 = new Slot0Configs();
        slot0.kS = 0.0;
        slot0.kV = 0.1;
        slot0.kA = 0.0;
        slot0.kG = 0.0;
        slot0.kP = 80.0;

        lifterConfig.MotorOutput
            .withNeutralMode(NeutralModeValue.Brake)
            .withInverted(InvertedValue.Clockwise_Positive);
        lifterConfig.Feedback
            .withSensorToMechanismRatio(Constants.Arm.GEAR_RATIO);
        lifterConfig.MotionMagic
            .withMotionMagicJerk(9999.0)
            .withMotionMagicAcceleration(4.5)
            .withMotionMagicCruiseVelocity(2.0 / 2.0); // RPS
        lifterConfig.CurrentLimits
            .withStatorCurrentLimitEnable(true)
            .withStatorCurrentLimit(70.0)
            .withSupplyCurrentLimitEnable(true)
            .withSupplyCurrentLimit(50.0);
        lifterConfig.Slot0 = slot0;
        
        this.roller.getConfigurator().apply(rollerConfig);
        this.lifter.getConfigurator().apply(lifterConfig);
    }

    // ---------- Method ---------- //
    public void resetRelativeFromAbsolute() {
        this.lifter.setPosition(0.5);
        isZeroed = true;
    }

    public void offsetArm(double v) {
        this.offsetRad += v;
    }

    public void setCoastEnabled(boolean coast) {
        this.lifter.setNeutralMode(coast ? NeutralModeValue.Coast : NeutralModeValue.Brake);
    }

    public static void setState(LifterState lifter, RollerState roller) {
        lifterState = lifter;
        rollerState = roller;
    }

    @Override
    public void periodic() {
        if (this.wantOffsetArmPositive.get()) this.offsetArm(Units.degreesToRadians(0.1));
        if (this.wantOffsetArmNegative.get()) this.offsetArm(-Units.degreesToRadians(0.1));

        boolean atStartOfAuto = (RobotState.isAutonomous() && 15.0 - DriverStation.getMatchTime() < 0.75);
        this.statorCurrent.refresh();

        boolean debouncedHasCoral = this.coralCurrentDebouncer.calculate(this.undebouncedHasObject());
        boolean debouncedHasAlgae = this.algaeCurrentDebouncer.calculate(this.undebouncedHasObject());
        hasObject = atStartOfAuto || (rollerState == RollerState.algeaIdle ? debouncedHasAlgae : debouncedHasCoral);

        if (!isZeroed || !Elevator.isZeroed) return;

        double gravityFeedforward = Constants.Arm.POSITION_DEPENDENT_KG * Math.sin(this.getPosition());

        this.roller.set(atStartOfAuto ? RollerState.fastIdle.value : rollerState.value);
        this.lifter.setControl(
            new MotionMagicVoltage(Units.radiansToRotations(this.getDesiredPosition() - this.offsetRad)).withSlot(0)
                .withFeedForward(gravityFeedforward));
    }

    // ---------- Function ---------- //
    public double getPosition() {
        return Units.rotationsToRadians(this.lifter.getPosition().getValueAsDouble()) + this.offsetRad;
    }

    public boolean isInsideFrame() {
        return Math.abs(MathUtil.angleModulus(this.getPosition())) < Constants.Arm.SAFE_INSIDE_ROBOT_ANGLE ||
            Math.abs(MathUtil.angleModulus(this.getPosition())) > (Math.PI - Constants.Arm.SAFE_INSIDE_ROBOT_ANGLE);
    }

    public boolean atSafeReefDistance() {
        return Swerve.getInstance().getPose().getTranslation()
            .getDistance(MathUtils.mirrorIfRed(Constants.Field.BLUE_REEF_CENTER)) > Constants.Arm.SAFE_DISTANCE_FROM_REEF_CENTER;
    }

    public boolean atSafeBargeDistance() {
        return Swerve.getInstance().getPose().getX() < Constants.Field.FIELD_X_SIZE / 2.0 - Constants.Arm.SAFE_BARGE_DISTANCE ||
            Swerve.getInstance().getPose().getX() > Constants.Field.FIELD_X_SIZE / 2.0 + Constants.Arm.SAFE_BARGE_DISTANCE;
    }

    public boolean atSafeProcessorDistance() {
        return Swerve.getInstance().getPose().getY() > Constants.Field.SAFE_WALL_DISTANCE && Swerve.getInstance().getPose().getY() < (Constants.Field.FIELD_Y_SIZE - Constants.Field.SAFE_WALL_DISTANCE);
    }

    public boolean atSafePlacementDistance() {
        return Swerve.getInstance().getPose().getTranslation().getDistance(MathUtils.mirrorIfRed(Constants.Field.BLUE_REEF_CENTER)) > Constants.Arm.SAFE_PLACEMENT_DISANCE;
    }

    public boolean atSetpoint() {
        return Math.abs(this.getDesiredPosition() - this.getPosition()) < Constants.Arm.SETPOINT_THRESHOLD;
    }

    public boolean undebouncedHasObject() {
        return this.statorCurrent.getValueAsDouble() > (rollerState == RollerState.idle ?
            Constants.Arm.IDEL_CURRENT_DRAW : Constants.Arm.CURRENT_DRAW);
    }

    public Side getCloserToReef() {
        Rotation2d directionTowardReefCenter = MathUtils.mirrorIfRed(Constants.Field.BLUE_REEF_CENTER).minus(Swerve.getInstance().getPose().getTranslation()).getAngle();
        Rotation2d directionTowardRight = Swerve.getInstance().getPose().getRotation().rotateBy(Rotation2d.kCW_90deg);

        double angle = Math.acos(directionTowardReefCenter.getCos() * directionTowardRight.getCos() + directionTowardReefCenter.getSin() * directionTowardRight.getSin());
        assert angle >= 0.0; // the math works out this way

        if (Math.PI / 2.0 - Constants.Arm.DEADZONE_ANGLE < angle && angle < Math.PI / 2.0 + Constants.Arm.DEADZONE_ANGLE) {
            return Side.Neither;
        } else if (angle < Math.PI / 2.0) {
            return Side.Right;
        } else {
            return Side.Left;
        }
    }

    public Side getCloserToBarge() {
        Pose2d estimatedPose = Swerve.getInstance().getPose();
        boolean isOnBlue = !(estimatedPose.getX() > (Constants.Field.FIELD_X_SIZE / 2.0));
        double rotation = estimatedPose.getRotation().getRadians();

        if ((rotation < Math.PI && rotation > Math.PI - Constants.Arm.DEADZONE_ANGLE) ||
            (rotation > -Math.PI && rotation < -Math.PI + Constants.Arm.DEADZONE_ANGLE) ||
            (rotation > -Constants.Arm.DEADZONE_ANGLE && rotation < Constants.Arm.DEADZONE_ANGLE)
        ) {
            return Side.Neither;
        } else if ((isOnBlue && rotation > 0.0) || (!isOnBlue && rotation < 0.0)) {
            return Side.Right;
        } else {
            return Side.Left;
        }
    }

    public Side getCloserToProcessor() {
        Pose2d estimatedPose = Swerve.getInstance().getPose();
        boolean isOnBlue = !(estimatedPose.getX() > (Constants.Field.FIELD_X_SIZE / 2.0));
        double rotation = estimatedPose.getRotation().getRadians();

        if ((rotation < (Math.PI / 2.0 + Constants.Arm.DEADZONE_ANGLE) && rotation > (Math.PI / 2.0 - Constants.Arm.DEADZONE_ANGLE)) ||
            (rotation > (-Math.PI / 2.0 - Constants.Arm.DEADZONE_ANGLE) && rotation < (-Math.PI / 2.0 + Constants.Arm.DEADZONE_ANGLE))
        ) {
            return Side.Neither;
        } else if ((isOnBlue && rotation < Math.PI / 2.0 && rotation > -Math.PI / 2.0) ||
            (!isOnBlue && (rotation > Math.PI / 2.0 || rotation < -Math.PI / 2.0))
        ) {
            return Side.Right;
        } else {
            return Side.Left;
        }
    }

    public double getPositionFromAngle(double angle, boolean respectReef) {
        List<Double> positions = Stream
            .of(
                MathUtils.wrapTo0_2PI(angle),
                MathUtils.wrapTo0_2PI(angle) - 2.0 * Math.PI)
            .filter(pos -> pos >= Constants.Arm.ALLOWED_OPERATING_MIN_RADIANS && pos <= Constants.Arm.ALLOWED_OPERATING_MAX_RADIANS)
            .collect(Collectors.toList());
        double actualArmPosition = this.getPosition();
        Side closeSide = this.getCloserToReef();

        double p;
        if (positions.size() == 1) {
            p = positions.get(0);
        } else if (respectReef) {
            switch (closeSide) {
                case Neither:
                    p = positions.stream()
                        .min(Comparator.comparingDouble(pos -> Math.abs(pos - actualArmPosition)))
                        .orElse(positions.get(0));
                    break;

                case Right:
                    if (actualArmPosition < Math.PI / 2.0) p = positions.get(1);
                    else p = positions.get(0);

                case Left:
                    if (actualArmPosition > -Math.PI / 2.0) p = positions.get(0);
                    else p = positions.get(1);

                default:
                    p = positions.get(0);
            } 
        } else {
            p = positions.stream()
                .min(Comparator.comparingDouble(pos -> Math.abs(pos - actualArmPosition)))
                .orElse(positions.get(0));
        }

        boolean notAtSafeReefDistance = !this.atSafeReefDistance();

        if (actualArmPosition > Math.PI / 2.0 && p < Math.PI / 2.0 &&
            closeSide == Side.Right && notAtSafeReefDistance
        ) {
            this.isStuck = true;
            p = Math.min(p, Math.PI - Constants.Arm.SAFE_INSIDE_ROBOT_ANGLE);
        } else if (actualArmPosition < -Math.PI / 2.0 && p > -Math.PI / 2.0 &&
            closeSide == Side.Left && notAtSafeReefDistance
        ) {
            this.isStuck = true;
            p = Math.min(p, -Math.PI + Constants.Arm.SAFE_INSIDE_ROBOT_ANGLE);
        } else {
            this.isStuck = false;
        }

        double actualElevatorHeight = Elevator.getInstance().getHeight();

        double limit = Intake.lifterState == Intake.LifterState.Down && Intake.getInstance().atSetpoint() ?
            elevatorToArmWhenIntakeDown.get(actualElevatorHeight) :
            elevatorToArm.get(actualElevatorHeight);
        
        if (MathUtil.isNear(Math.PI, limit, 0.0001)) return p;
        else if (actualArmPosition < 0.0) return Math.max(-Math.PI - limit, Math.min(-Math.PI + limit, p));
        else return Math.max(Math.PI - limit, Math.min(Math.PI + limit, p));
    }

    public double getDesiredPosition() {
        if (this.lastUpdatedTick == Robot.tick) return this.lastCachedValue;
        double value;
        double startTime = System.currentTimeMillis();

        value = this.getPositionFromAngle(
            lifterState.desiredAngle(
                this.getPosition(),
                getCloserToReef(),
                getCloserToBarge(),
                getCloserToProcessor()),
            lifterState.mirrorType != MirrorType.ActuallyFixedAngle);
        double milliseconds = System.currentTimeMillis() - startTime;

        this.lastUpdatedTick = Robot.tick;
        this.lastCachedValue = value;

        if (milliseconds > 5.0) System.out.println("Desired position took " + milliseconds + " ms");
        return value;
    }

    public double getCloseClampedPosition() {
        double value = this.encoder.get() - Constants.Arm.ENCODER_OFFSET_ROTATION;

        while (value < -0.5) value += 1.0;
        while (value > 0.5) value -= 1.0;

        double allowedOffsetArmRotations = 12.0 / 360.0;

        if (Math.abs(value - 0.5) < allowedOffsetArmRotations) return 0.5;
        else if (Math.abs(value + 0.5) < allowedOffsetArmRotations) return -0.5;
        else return value;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addBooleanProperty("HasObject", () -> hasObject, null);
        builder.addBooleanProperty("AtSafeReefDistance", () -> this.atSafeReefDistance(), null);
        builder.addStringProperty("RollerState", () -> rollerState.toString(), null);
        builder.addStringProperty("ArmState", () -> lifterState.toString(), null);
        builder.addDoubleProperty("RollerCurrent", () -> this.statorCurrent.getValueAsDouble(), null);
        builder.addDoubleProperty("Angle", () -> Units.radiansToRotations(this.getPosition()), null);
        builder.addDoubleProperty("MotionMagic Setpoint", () -> this.lifter.getClosedLoopReference().getValueAsDouble(), null);
        builder.addDoubleProperty("Voltage", () -> this.lifter.getMotorVoltage().getValueAsDouble(), null);
    }

    // ---------- Simulation ---------- //
    private final TalonFXSimState simState = this.lifter.getSimState();
    private final StructPublisher<Pose3d> armComponent = NetworkTableInstance.getDefault()
        .getStructTopic("Component/Arm",  Pose3d.struct).publish();
    private final SingleJointedArmSim armSim = new SingleJointedArmSim(
        DCMotor.getFalcon500(1),
        Constants.Arm.GEAR_RATIO,
        SingleJointedArmSim.estimateMOI(0.6, 2.5),
        0.6,
        -Math.PI,
        Math.PI,
        false,
        0.0);
    
    public Pose3d getArmComponentPose() {
        return Elevator.getInstance().getCarriageComponentPose()
            .plus(new Transform3d(0.0, 0.0, 0.300355,
                new Rotation3d(this.getPosition() + Math.PI, 0.0, 0.0)));
    }

    public void simulationUpdate() {
        this.armSim.setInputVoltage(this.simState.getMotorVoltage());
        this.armSim.update(0.020);

        double angleRad = this.armSim.getAngleRads() * Constants.Arm.GEAR_RATIO / (2.0 * Math.PI);
        double velocity = this.armSim.getVelocityRadPerSec() * Constants.Arm.GEAR_RATIO / (2.0 * Math.PI);

        this.simState.setRawRotorPosition(angleRad);
        this.simState.setRotorVelocity(velocity);

        this.armComponent.accept(this.getArmComponentPose());
    }
}
