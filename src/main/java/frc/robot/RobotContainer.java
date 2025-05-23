package frc.robot;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class RobotContainer {
	private final Intake intake = new Intake();

	public RobotContainer() {
		Shuffleboard.getTab("Intake").add(this.intake);
	}

	public Command getAutonomousCommand() {
		return null;
	}
}
