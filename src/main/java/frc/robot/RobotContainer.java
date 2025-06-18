package frc.robot;

import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.ElevatorCmd;
import frc.robot.subsystems.Elevator;

public class RobotContainer {
	private final PS5Controller controller = new PS5Controller(0);
	private final Elevator elevatorSubsystem = new Elevator();
	
	public RobotContainer() {
		// this.elevatorSubsystem.setDefaultCommand(new ElevatorCmd(this.elevatorSubsystem, this.controller::getLeftY));
		Shuffleboard.getTab("Elevator").add(this.elevatorSubsystem);
	}

	public Command getAutonomousCommand() {
		return null;
	}
}
