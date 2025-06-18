package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

public class ElevatorCmd extends Command {
	private final Elevator elevator;
	private final Supplier<Double> voltage;

	public ElevatorCmd(Elevator elevator, Supplier<Double> voltage) {
		this.elevator = elevator;
		this.voltage = voltage;
		this.addRequirements(this.elevator);
	}

	@Override
	public void execute() {
		this.elevator.setVoltage(this.voltage.get() * 1.5);
	}

	@Override
	public void end(boolean interrupted) {
		this.elevator.setZeroVoltage();
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
