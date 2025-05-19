package frc.robot;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;
public class RobotContainer {
	private final Arm arm = new Arm();

	public RobotContainer() {
		Shuffleboard.getTab("Arm").add(this.arm);
	}

	public Command getAutonomousCommand() {
		return null;
	}
}
