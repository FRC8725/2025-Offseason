package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.lib.helpers.IDashboardProvider;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class ProfiledPid_Test extends SubsystemBase implements IDashboardProvider {
    public final TalonFX motor = new TalonFX(1);

    private double feedbackVoltage;
    private double feedforwardVoltage;

    private double down = 10.0;// DOTO
    private double up = 50.0;// DOTO

    private static double max = 50.0;
    private static double min = 0.0;

    private final TrapezoidProfile.Constraints m_constraints = new TrapezoidProfile.Constraints(12,
            12);
    private final ProfiledPIDController pidController = new ProfiledPIDController(5, 0, 0, m_constraints);
    private final ArmFeedforward ArmFeedforward = new ArmFeedforward(0.11757, 0.0051374, 0.11375, 0.0011209);

    private final VoltageOut voltagRequire = new VoltageOut(0.0);
    private final SysIdRoutine sysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(Volts.of(2).per(Second), Volts.of(5),
                    null, (state) -> SignalLogger.writeString("state", state.toString())),
            new SysIdRoutine.Mechanism(
                    (volts) -> {
                        this.motor.setControl(voltagRequire.withOutput(volts.in(Volts)));
                    },
                    null,
                    this));

    public ProfiledPid_Test() {
        this.registerDashboard();
        this.motor.clearStickyFaults();
        this.motor.setNeutralMode(NeutralModeValue.Brake);
        this.motor.setPosition(0.0);
    }

    public double encoder() {
        return this.motor.getPosition().getValueAsDouble();
    }

    public void setVoltage(double speed) {
        speed = MathUtil.clamp(speed, -12, 12);
        this.motor.setVoltage(speed);
        SmartDashboard.putNumber("Total Voltage", speed);
    }

    public void stop() {
        this.motor.stopMotor();
    }

    public Command goalto(double goalPositionSupplier) {
        return run(() -> {
            feedbackVoltage = pidController.calculate(this.encoder());
            feedforwardVoltage = ArmFeedforward.calculate(this.encoder(), pidController.getSetpoint().velocity);
            setVoltage(feedbackVoltage + feedforwardVoltage);
            SmartDashboard.putNumber("goalPositionSupplier", goalPositionSupplier);
            SmartDashboard.putNumber("feedbackVoltage", feedbackVoltage);
            SmartDashboard.putNumber("feedforwardVoltag", feedforwardVoltage);
        });
    }

    public Command moveToPositionCommand(double goalPositionSupplier) {
        return Commands.sequence(
                restandset(goalPositionSupplier),
                goalto(goalPositionSupplier)
                        .until(() -> pidController.atGoal()))
                .withTimeout(7);
    }

    public Command restandset(double goalPositionSupplier) {
        return Commands.runOnce(() -> {
            pidController.reset(this.encoder());
            pidController.setGoal(goalPositionSupplier);
        });
    }

    public Command down() {
        return moveToPositionCommand(down)
                .finallyDo(this::stop);
    }

    public Command up() {
        return moveToPositionCommand(up)
                .finallyDo(this::stop);
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return this.sysIdRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return this.sysIdRoutine.dynamic(direction);
    }

    public Command startCommand() {
        return Commands.runOnce(SignalLogger::start);
    }

    public Command stopCommand() {
        return Commands.runOnce(SignalLogger::stop);
    }

    public Command sysIdElevatorTest() {
        return Commands.sequence(
                this.sysIdQuasistatic(SysIdRoutine.Direction.kForward)
                        .raceWith(new WaitUntilCommand(() -> this.encoder() > this.max - 0.15)),
                new WaitCommand(1.5),

                this.sysIdQuasistatic(SysIdRoutine.Direction.kReverse)
                        .raceWith(new WaitUntilCommand(() -> this.encoder() < this.min + 0.15)),
                new WaitCommand(1.5),

                this.sysIdDynamic(SysIdRoutine.Direction.kForward)
                        .raceWith(new WaitUntilCommand(() -> this.encoder() > this.max - 0.15)),
                new WaitCommand(1.5),

                this.sysIdDynamic(SysIdRoutine.Direction.kReverse)
                        .raceWith(new WaitUntilCommand(() -> this.encoder() < this.min + 0.15)));
    }

    public boolean atgoal() {
        return pidController.atGoal();
    }

    @Override
    public void putDashboard() {
        SmartDashboard.putNumber("Encoder", this.encoder());
        SmartDashboard.putNumber("Pidspeed", this.feedbackVoltage + this.feedforwardVoltage);
        SmartDashboard.putBoolean("atGoal", pidController.atGoal());
        SmartDashboard.putNumber("Setpoint Position", pidController.getSetpoint().position);
        SmartDashboard.putNumber("Setpoint Velocity", pidController.getSetpoint().velocity);

    }
}
