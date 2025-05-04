package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.ElevatorConstans;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorCmd extends Command {
	private final ElevatorSubsystem elevatorSubsystem;
	private final Supplier<Double> speed;
	private double position = 0.0;

	public ElevatorCmd(ElevatorSubsystem elevatorSubsystem, Supplier<Double> speed) {
		this.elevatorSubsystem = elevatorSubsystem;
		this.speed = speed;
		this.addRequirements(this.elevatorSubsystem);
	}

	@Override
	public void initialize() {}

	@Override
	public void execute() {
		double delta = this.speed.get() * 0.01;
		position += delta;
		position = Math.max(ElevatorConstans.MIN_ROTATIONS, Math.min(ElevatorConstans.MAX_ROTATIONS, position));
		this.elevatorSubsystem.moveToPosition(position);
	}

	@Override
	public void end(boolean interrupted) {
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
