package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.Constants;
import frc.robot.lib.helpers.IDashboardProvider;

public class ElevatorSubsystem extends SubsystemBase implements IDashboardProvider {
	private final TalonModule left = new TalonModule(1, false, true);
	private final TalonModule right = new TalonModule(2, false, true);
	private final Follower follower = new Follower(0, true);

	private final TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(500,
			250);
	private final ProfiledPIDController pidController = new ProfiledPIDController(0.0, 0.0, 0.0, constraints);// TODO
	private final ElevatorFeedforward elevatorFeedforward = new ElevatorFeedforward(0.0, 0.3, 0.0, 0.0);

	private final DoubleSubscriber kP = NetworkTableInstance.getDefault().getTable("SmartDashboard")
			.getDoubleTopic("Elevator/PID/p").subscribe(0.0);
	private final DoubleSubscriber kI = NetworkTableInstance.getDefault().getTable("SmartDashboard")
			.getDoubleTopic("Elevator/PID/i").subscribe(0.0);
	private final DoubleSubscriber kD = NetworkTableInstance.getDefault().getTable("SmartDashboard")
			.getDoubleTopic("Elevator/PID/d").subscribe(0.0);
	// private final DoubleSubscriber setpoint =
	// NetworkTableInstance.getDefault().getTable("SmartDashboard")
	// .getDoubleTopic("Elevator/PID/setpoint").subscribe(0.0);

	private final VoltageOut voltageRequire = new VoltageOut(0.0);
	private final SysIdRoutine sysIdRoutine = new SysIdRoutine(
			new SysIdRoutine.Config(Volts.of(2).per(Second), Volts.of(5), null,
					(state) -> SignalLogger.writeString("state", state.toString())),
			new SysIdRoutine.Mechanism(
					(volts) -> {
						this.left.setControl(voltageRequire.withOutput(volts.in(Volts)));
						this.right.setControl(voltageRequire.withOutput(-volts.in(Volts)));
					},
					null,
					this));

	// private final double VELOCITY_CONVERSION = Units.inchesToMeters(0.938) * 2.0
	// * Math.PI / 8.0;
	// private final double POSITION_CONVERSION = 1.0 / 8.0;

	// private final ElevatorSim elevatorSim = new ElevatorSim(// 模擬升降馬達與負載行為
	// DCMotor.getKrakenX60(2), // 使用兩顆馬達
	// 12.6, // 齒輪比
	// 20, // 質量
	// Units.inchesToMeters(1), // 滑輪半徑
	// 0.0, // 起始位置
	// Units.inchesToMeters(24),
	// true,
	// 0.0);
	// // private final MechanismLigament2d ligament;
	// private final StructPublisher<Pose3d> elevatorStage =
	// NetworkTableInstance.getDefault()
	// .getStructTopic("AdvantageScope/ElevatorStage", Pose3d.struct).publish();
	// private final StructPublisher<Pose3d> elevatorCarriage =
	// NetworkTableInstance.getDefault()
	// .getStructTopic("AdvantageScope/ElevatorCarriage", Pose3d.struct).publish();
	// private double simVelocity = 0.0;

	private final double MIN_ROTATIONS = 0.0;
	private final double MAX_ROTATIONS = 4.9445068359375;
	private double feedforwardVoltage = 0.0;
	private double feedbackVoltage = 0.0;

	public ElevatorSubsystem() {
		super("Elevator");
		registerDashboard();
		// this.ligament = ligament;
		pidController.setTolerance(Constants.ElevatorConstants.TOLERANCE);
		left.setNeutralMode(NeutralModeValue.Brake);
		right.setNeutralMode(NeutralModeValue.Brake);

		SmartDashboard.putData("Elevator/PID", new Sendable() {

			@Override
			public void initSendable(SendableBuilder builder) {
				builder.setSmartDashboardType("PIDController");
				builder.addDoubleProperty("p", () -> kP.get(), null);
				builder.addDoubleProperty("i", () -> kI.get(), null);
				builder.addDoubleProperty("d", () -> kD.get(), null);
			}
		});
	}

	public double getPosition() {
		return this.left.getPosition().getValueAsDouble();
	}

	public void setVoltage(double speed) {
		speed = MathUtil.clamp(speed, -12, 12);
		this.left.setVoltage(speed);
		this.right.setVoltage(speed);
	}

	public Command restandset(double goalPositionSupplier) {
		return runOnce(() -> {
			pidController.reset(this.getPosition());
			pidController.setGoal(goalPositionSupplier);
		});
	}

	public Command goal(double goalPositionSluppier) {
		return run(() -> {
			feedbackVoltage = pidController.calculate(this.getPosition());
			feedforwardVoltage = elevatorFeedforward.calculate(this.getPosition(),
					pidController.getSetpoint().velocity);
			setVoltage(feedbackVoltage + feedforwardVoltage);
			SmartDashboard.putNumber("GoalPositionSluppier", goalPositionSluppier);
		});
	}

	public Command moveToPositionCommand(double goalPosition) {
		return Commands.sequence(
				restandset(goalPosition),
				goal(goalPosition).until(() -> pidController.atGoal()).withTimeout(3));
	}

	public Command down() {
		return moveToPositionCommand(MIN_ROTATIONS)
				.finallyDo(this::stop);
	}

	public Command up() {
		return moveToPositionCommand(MAX_ROTATIONS)
				.finallyDo(this::stop);
	}

	public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
		return this.sysIdRoutine.quasistatic(direction);
	}

	public Command sysIdDynamic(SysIdRoutine.Direction direction) {
		return this.sysIdRoutine.dynamic(direction);
	}

	public Command start() {
		return Commands.runOnce(SignalLogger::start);
	}

	public Command stop() {
		return Commands.runOnce(SignalLogger::stop);
	}

	public Command sysIDElevator() {
		return Commands.sequence(
				this.sysIdQuasistatic(SysIdRoutine.Direction.kForward)
						.raceWith(new WaitUntilCommand(() -> this.getPosition() > this.MAX_ROTATIONS - 0.15)),
				new WaitCommand(2),

				this.sysIdQuasistatic(SysIdRoutine.Direction.kReverse)
						.raceWith(new WaitUntilCommand(() -> this.getPosition() < this.MIN_ROTATIONS + 0.15)),
				new WaitCommand(2),

				this.sysIdDynamic(SysIdRoutine.Direction.kForward)
						.raceWith(new WaitUntilCommand(() -> this.getPosition() > this.MAX_ROTATIONS - 0.15)),
				new WaitCommand(2),

				this.sysIdDynamic(SysIdRoutine.Direction.kReverse)
						.raceWith(new WaitUntilCommand(() -> this.getPosition() < this.MIN_ROTATIONS + 0.15)),
				new WaitCommand(2));
	}

	@Override
	public void putDashboard() {
		SmartDashboard.putNumber("Position", this.getPosition());
		SmartDashboard.putNumber("FeedbackVoltage", feedbackVoltage);
		SmartDashboard.putNumber("FeedforwardVoltage", feedforwardVoltage);
	}
}