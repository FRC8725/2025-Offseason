package frc.robot.subsystems;

import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {
  private final TalonModule elevator = new TalonModule(18, false, true);

  private final DoubleSubscriber kP = NetworkTableInstance.getDefault().getTable("SmartDashboard")
      .getDoubleTopic("elevator/PID/p").subscribe(0.0);
  private final DoubleSubscriber kI = NetworkTableInstance.getDefault().getTable("SmartDashboard")
      .getDoubleTopic("elevator/PID/i").subscribe(0.0);
  private final DoubleSubscriber kD = NetworkTableInstance.getDefault().getTable("SmartDashboard")
      .getDoubleTopic("elevator/PID/d").subscribe(0.0);
  private final DoubleSubscriber setpoint = NetworkTableInstance.getDefault().getTable("SmartDashboard")
      .getDoubleTopic("elevator/PID/setpoint").subscribe(0.0);

  public ElevatorSubsystem() {

  }
}
