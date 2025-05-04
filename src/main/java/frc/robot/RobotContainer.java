package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.ElevatorCmd;
import frc.robot.subsystems.ElevatorSubsystem;

public class RobotContainer {
	private final Joystick joystick = new Joystick();
	private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
	
	public RobotContainer() {
		this.elevatorSubsystem.setDefaultCommand(
			new ElevatorCmd(this.elevatorSubsystem, this.joystick::getElevatorSpeed));
	}

	public Command getAutonomousCommand() {
		return null;
	}
}
