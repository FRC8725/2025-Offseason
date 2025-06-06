package frc.robot;

import frc.robot.commands.ElevatorCmd;
import frc.robot.subsystems.ElevatorSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class RobotContainer {
	private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();

	private final Controller controller = new Controller();

	private final ElevatorCmd elevatorCommand = new ElevatorCmd(elevatorSubsystem, controller);

	public RobotContainer() {
		this.elevatorSubsystem.setDefaultCommand(elevatorCommand);
		this.configureBindings();
	}

	private void configureBindings() {
		this.controller.Up()
				.whileTrue(elevatorSubsystem.up());

		this.controller.Down()
				.whileTrue(elevatorSubsystem.down());

		this.controller.Start()
				.whileTrue(elevatorSubsystem.start());

		this.controller.Stop()
				.whileTrue(elevatorSubsystem.stop());

		this.controller.Test()
				.whileTrue(elevatorSubsystem.sysIDElevator()
						.alongWith(elevatorSubsystem.start()));
	}

	public Command getAutonomousCommand() {
		return null;
	}
}