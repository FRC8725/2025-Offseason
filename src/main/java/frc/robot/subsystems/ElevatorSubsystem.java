package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volt;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class ElevatorSubsystem extends SubsystemBase {
  private final TalonModule left = new TalonModule(15, false, true);
  private final TalonModule right = new TalonModule(16, false, true);
  private final Follower follower = new Follower(15, true);

  private final PIDController pid = new PIDController(0, 0, 0); // TODO
  private final ElevatorFeedforward elevatorFeedforward = new ElevatorFeedforward(0.0, 0.0, 0.0, 0.0);

  private final DoubleSubscriber kP = NetworkTableInstance.getDefault().getTable("SmartDashboard")
      .getDoubleTopic("Elevator/PID/p").subscribe(0.0);
  private final DoubleSubscriber kI = NetworkTableInstance.getDefault().getTable("SmartDashboard")
      .getDoubleTopic("Elevator/PID/i").subscribe(0.0);
  private final DoubleSubscriber kD = NetworkTableInstance.getDefault().getTable("SmartDashboard")
      .getDoubleTopic("Elevator/PID/d").subscribe(0.0);
  private final DoubleSubscriber setpoint = NetworkTableInstance.getDefault().getTable("SmartDashboard")
      .getDoubleTopic("Elevator/PID/setpoint").subscribe(0.0);

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

  private final double VELOCITY_CONVERSION = Units.inchesToMeters(0.938) * 2.0 * Math.PI / 8.0;
  private final double POSITION_CONVERSION = 1.0 / 8.0;

  private final ElevatorSim elevatorSim = new ElevatorSim(// 模擬升降馬達與負載行為
      DCMotor.getKrakenX60(2), // 使用兩顆馬達
      12.6, // 齒輪比
      20, // 質量
      Units.inchesToMeters(1), // 滑輪半徑
      0.0, // 起始位置
      Units.inchesToMeters(24),
      true,
      0.0);
  private final MechanismLigament2d ligament;
  private final StructPublisher<Pose3d> elevatorStage = NetworkTableInstance.getDefault()
      .getStructTopic("AdvantageScope/ElevatorStage", Pose3d.struct).publish();
  private final StructPublisher<Pose3d> elevatorCarriage = NetworkTableInstance.getDefault()
      .getStructTopic("AdvantageScope/ElevatorCarriage", Pose3d.struct).publish();
  private double simVelocity = 0.0;

  private final double MIN_ROTATIONS = 0.0;
  private final double MAX_ROTATIONS = 4.9445068359375;
  private final double TOLERANCE = 0.08;
  private final double feedfowardVoltage = 0.0;
  private final double feedbackVoltage = 0.0;

  public ElevatorSubsystem(MechanismLigament2d ligament) {
    super("Elevator");
    this.ligament = ligament;
    pid.setTolerance(TOLERANCE);
    right.clearStickyFaults();
    left.clearStickyFaults();
    left.setPosition(0.0);
    right.setControl(follower);
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
}
