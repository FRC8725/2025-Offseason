package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveCmd extends Command {
	private final SwerveSubsystem swerveSubsystem;
	private final Supplier<Double> xSpeed, ySpeed, rSpeed;

	public SwerveCmd(
		SwerveSubsystem swerveSubsystem,
		Supplier<Double> xSpeed, Supplier<Double> ySpeed, Supplier<Double> rSpeed) {
		this.swerveSubsystem = swerveSubsystem;
		this.xSpeed = xSpeed;
		this.ySpeed = ySpeed;
		this.rSpeed = rSpeed;
		this.addRequirements(this.swerveSubsystem);
	}

	@Override
	public void initialize() {}

	@Override
	public void execute() {
		this.swerveSubsystem.drive(this.xSpeed.get(), this.ySpeed.get(), this.rSpeed.get(), false);
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
