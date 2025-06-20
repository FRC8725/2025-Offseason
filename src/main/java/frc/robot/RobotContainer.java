package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class RobotContainer {
	private final XboxController controller = new XboxController(0);
	private final Intake intake = new Intake(this.controller::getAButton);
	
	public RobotContainer() {
		Shuffleboard.getTab("Intake").add(this.intake);
	}

	public Command getAutonomousCommand() {
		return null;
	}
}
