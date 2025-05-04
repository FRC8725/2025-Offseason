package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.ElevatorConstans;
import frc.robot.lib.motor.TalonModule;
import frc.robot.lib.subsystems.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {
    private final TalonModule left = new TalonModule(0, false, true);
    private final TalonModule right = new TalonModule(0, false, true);
    private final Follower follower = new Follower(left.getDeviceID(), true);
    private final MotionMagicVoltage request = new MotionMagicVoltage(0.0);


    private final VoltageOut voltReq = new VoltageOut(0.0);
    private final SysIdRoutine sysIdRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // Use default ramp rate (1 V/s)
            Volts.of(4), // Reduce dynamic step voltage to 4 to prevent brownout
            null,        // Use default timeout (10 s)
            (state) -> SignalLogger.writeString("elevator-state", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            (volts) -> this.left.setControl(this.voltReq.withOutput(volts.in(Volts))),
            null,
            this
        )
    );

    public ElevatorSubsystem() {
        super("Elevator", false);
        this.right.setControl(this.follower);
    }

    public void moveToPosition(double position) {
        this.left.setControl(this.request.withPosition(position));
    }

    public Command testSysId() {
        return Commands.sequence(
            this.sysIdQuasistatic(Direction.kForward)
                .raceWith(new WaitUntilCommand(() -> this.left.getPositionValue() > ElevatorConstans.MAX_ROTATIONS)),
            new WaitCommand(1.0),
            this.sysIdQuasistatic(Direction.kReverse)
                .raceWith(new WaitUntilCommand(() -> this.left.getPositionValue() < ElevatorConstans.MIN_ROTATIONS)),
            new WaitCommand(1.0),
            this.sysIdDynamic(Direction.kForward)
                .raceWith(new WaitUntilCommand(() -> this.left.getPositionValue() > ElevatorConstans.MAX_ROTATIONS)),
            new WaitCommand(1.0),
            this.sysIdDynamic(Direction.kReverse)
                .raceWith(new WaitUntilCommand(() -> this.left.getPositionValue() < ElevatorConstans.MIN_ROTATIONS)));
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return this.sysIdRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return this.sysIdRoutine.dynamic(direction);
    }

    @Override
    public void periodic() {}

    @Override
    public void putDashboard() {
        SmartDashboard.putNumber("Elevator/left Velocity", this.left.getVelocityValue());
        SmartDashboard.putNumber("Elevator/left Acceleration", this.left.getAcceleration().getValueAsDouble());
        SmartDashboard.putNumber("Elevator/right Velocity", this.right.getVelocityValue());
        SmartDashboard.putNumber("Elevator/right Acceleration", this.right.getAcceleration().getValueAsDouble());
        
    }
}
