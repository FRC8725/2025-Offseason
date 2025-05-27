package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.SwerveCmd;
import frc.robot.subsystems.Swerve;

public class RobotContainer {
	private final Driver driver = new Driver();
	private final Swerve swerveSubsystem = new Swerve();

	public RobotContainer() {
		this.swerveSubsystem.setDefaultCommand(
			new SwerveCmd(this.swerveSubsystem, this.driver::getSpeeds));
		new Trigger(this.driver::getAButton).whileTrue(this.swerveSubsystem.sysIdTest());
	}
	
	public Command getAutonomousCommand() {
		return null;
	}
}
