package frc.robot;

import frc.robot.commands.ElevatorCmd;
import frc.robot.subsystems.ElevatorSubsystem;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class RobotContainer {
	private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();

	private final Controller controller = new Controller();

	private final ElevatorCmd elevatorCommand = new ElevatorCmd(elevatorSubsystem, controller);

	public RobotContainer() {
		//this.elevatorSubsystem.setDefaultCommand(elevatorCommand);
		this.configureBindings();
	}

	private void configureBindings() {
		this.controller.Up()
				.onTrue(elevatorSubsystem.up());

		this.controller.Down()
				.onTrue(elevatorSubsystem.down());

		this.controller.Start()
				.onTrue(Commands.runOnce(SignalLogger::start));

		this.controller.Stop()
				.onTrue(Commands.runOnce(SignalLogger::stop));

		this.controller.Test()
				.whileTrue(elevatorSubsystem.sysIDElevator());
	}

	public Command getAutonomousCommand() {
		return null;
	}
}