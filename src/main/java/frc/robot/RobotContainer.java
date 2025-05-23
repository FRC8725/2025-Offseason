package frc.robot;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;

public class RobotContainer {
	private final Elevator elevatorSubsystem = new Elevator();
	private final Arm arm = new Arm();
	
	public RobotContainer() {
		Shuffleboard.getTab("Elevator").add(this.elevatorSubsystem);
		Shuffleboard.getTab("Arm").add(this.arm);
	}

	public Command getAutonomousCommand() {
		return null;
	}
}
