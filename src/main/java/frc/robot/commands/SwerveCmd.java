package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;

public class SwerveCmd extends Command {
	private final Swerve swerveSubsystem;
	private final Supplier<ChassisSpeeds> getSpeeds;

	public SwerveCmd(Swerve swerveSubsystem, Supplier<ChassisSpeeds> getSpeeds) {
		this.swerveSubsystem = swerveSubsystem;
		this.getSpeeds = getSpeeds;
		this.addRequirements(this.swerveSubsystem);
	}

	@Override
	public void initialize() {}

	@Override
	public void execute() {
		this.swerveSubsystem.driveRobotRelative(this.getSpeeds.get());
	}

	@Override
	public void end(boolean interrupted) {
		this.swerveSubsystem.stopModules();
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
