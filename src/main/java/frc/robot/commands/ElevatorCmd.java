package frc.robot.commands;

import frc.robot.subsystems.ElevatorSubsystem;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;

public class ElevatorCmd extends Command {
	private final ElevatorSubsystem elevatorSubsystem;
	private final XboxController xboxController;

	public ElevatorCmd(ElevatorSubsystem elevatorSubsystem, XboxController xboxController) {
		this.elevatorSubsystem = elevatorSubsystem;
		this.xboxController = xboxController;
		this.addRequirements(this.elevatorSubsystem);
	}

	@Override
	public void initialize() {
	}

	@Override
	public void execute() {
		double speed = MathUtil.applyDeadband(this.xboxController.getLeftY(), 0.05) * 1;
		this.elevatorSubsystem.setVoltage(speed);
	}

	@Override
	public void end(boolean interrupted) {
		this.elevatorSubsystem.stop();
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}