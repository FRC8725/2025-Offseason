package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.SwerveCmd;
import frc.robot.joysticks.Driver;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class RobotContainer {
	private final Driver driver = new Driver();
	private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
	private final VisionSubsystem visionSubsystem = new VisionSubsystem(
		this.swerveSubsystem::addVisionMeasurement, this.swerveSubsystem::getPose);

	public RobotContainer() {
		this.swerveSubsystem.setDefaultCommand(
			new SwerveCmd(
				this.swerveSubsystem,
				this.driver::getXDesiredSpeed, this.driver::getYDesiredSpeed, this.driver::getRDesiredSpeed));
	}

	public void configBindings() {
		this.driver.resetGyro()
			.onTrue(Commands.runOnce(this.swerveSubsystem::resetGyro, this.swerveSubsystem));
		this.driver.resetPosition()
			.onTrue(Commands.runOnce(this.swerveSubsystem::resetSwerveEncoders, this.swerveSubsystem));
	}

	public Command getAutonomousCommand() {
		return null;
	}
}
