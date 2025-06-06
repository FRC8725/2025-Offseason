// Copyright (c) FIRST and other WPILib contributors.
package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Joysticks;
import frc.robot.Joysticks.AlignMode;
import frc.robot.subsystems.Swerve;

public class DriveCmd extends Command {
	private final Swerve swerve;
	private final Supplier<Joysticks.DriveInputs> driveInputs;
	
	private final PIDController xPid = new PIDController(0, 0, 0);
	private final PIDController yPid = new PIDController(0, 0, 0);
	private final PIDController rotatePid = new PIDController(0, 0, 0);

	private AlignMode lastAlignMode = AlignMode.None;

	public DriveCmd(Swerve swerve, Supplier<Joysticks.DriveInputs> driveInputs) {
		this.swerve = swerve;
		this.driveInputs = driveInputs;
		this.addRequirements(this.swerve);
	}

	@Override
	public void initialize() {}

	@Override
	public void execute() {
		Joysticks.DriveInputs inputs = this.driveInputs.get();
		if (DriverStation.getAlliance().get() == Alliance.Red) inputs = inputs.getRedFlipped();

		if (inputs.isNonZero() && !RobotState.isAutonomous()) {
			inputs.alignMode = AlignMode.None;
		}
		
		if (inputs.alignMode != this.lastAlignMode) {
			this.xPid.reset();
			this.yPid.reset();
			this.rotatePid.reset();
		}
		this.lastAlignMode = inputs.alignMode;

		if (inputs.alignMode == AlignMode.None) {
			this.swerve.driveRobotRelative(this.getSpeeds());
		} else if (inputs.alignMode == AlignMode.BargeAlign) {

		} else {
			
		}
	}

	@Override
	public void end(boolean interrupted) {
	}

	@Override
	public boolean isFinished() {
		return false;
	}

	public ChassisSpeeds getSpeeds() {
		double x = this.driveInputs.get().leftX;
		double y = this.driveInputs.get().leftY;
		double rot = this.driveInputs.get().rightX;

		double theta = Math.atan2(y, x);
		double r = Math.hypot(x, y);
		
		r = this.deadZone(r, this.driveInputs.get().deadZone);
		rot = this.deadZone(rot, this.driveInputs.get().deadZone);

		r = r * r;
		rot = rot * rot * Math.signum(rot);

		double xSpeed = r * Math.cos(theta) * Constants.Swerve.MAX_VELOCITY;
		double ySpeed = r * Math.sin(theta) * Constants.Swerve.MAX_VELOCITY;
		double rSpeed = rot * Constants.Swerve.MAX_ANGULAR_VELOCITY;

		return new ChassisSpeeds(xSpeed, ySpeed, rSpeed);
	}

	public double deadZone(double input, double deadZone) {
		if (Math.abs(input) < deadZone) return 0.0;
		else if (input > 1.0) return 1.0;
		else if (input < -1.0) return -1.0;
		else return (input - deadZone) / (1.0 - deadZone);
	}
}
