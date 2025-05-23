package frc.robot;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

public class RobotContainer {
	private final Elevator elevatorSubsystem = new Elevator();
	
	public RobotContainer() {
		Shuffleboard.getTab("Elevator").add(this.elevatorSubsystem);
	}

	public Command getAutonomousCommand() {
		return null;
	}
}
