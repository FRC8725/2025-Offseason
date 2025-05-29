package frc.robot;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;

public class RobotContainer {
	private final Swerve swerve = new Swerve();
	private final Vision vision = new Vision();
	private final Elevator elevator = new Elevator();
	private final Arm arm = new Arm();
	
	public RobotContainer() {
		Shuffleboard.getTab("Vision").add(this.vision);
		Shuffleboard.getTab("Elevator").add(this.elevator);
		Shuffleboard.getTab("Arm").add(this.arm);
	}

	public Command getAutonomousCommand() {
		return null;
	}
}
