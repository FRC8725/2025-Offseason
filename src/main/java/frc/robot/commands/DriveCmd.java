// Copyright (c) FIRST and other WPILib contributors.
package frc.robot.commands;

import java.util.Map;
import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Joysticks;
import frc.robot.Robot;
import frc.robot.Joysticks.AlignMode;
import frc.robot.lib.math.MathUtils;
import frc.robot.subsystems.Swerve;

public class DriveCmd extends Command {
	private final Swerve swerve;
	private final Supplier<Joysticks.DriveInputs> driveInputs;
	
	private final PIDController xPid = new PIDController(5.0, 0.0, 0.01);
	private final PIDController yPid = new PIDController(5.0, 0.0, 0.01);
	private final PIDController turnPid = new PIDController(6.0, 0.0, 0.04);

	private AlignMode lastAlignMode = AlignMode.None;

	public DriveCmd(Swerve swerve, Supplier<Joysticks.DriveInputs> driveInputs) {
		this.swerve = swerve;
		this.driveInputs = driveInputs;
		this.turnPid.enableContinuousInput(-Math.PI, Math.PI);
		this.addRequirements(this.swerve);
	}

	@Override
	public void initialize() {}

	@Override
	public void execute() {
		Joysticks.DriveInputs inputs = this.driveInputs.get();
		if (Robot.isRedAlliance) inputs = inputs.getRedFlipped();

		if (inputs.isNonZero() && !RobotState.isAutonomous()) {
			inputs.alignMode = AlignMode.None;
		}
		
		if (inputs.alignMode != this.lastAlignMode) {
			this.xPid.reset();
			this.yPid.reset();
			this.turnPid.reset();
		}
		this.lastAlignMode = inputs.alignMode;
		Swerve.getInstance().isAligned = false;

		if (inputs.alignMode == AlignMode.None) {
			this.swerve.driveRobotRelative(this.getSpeeds());
		} else if (inputs.alignMode == AlignMode.BargeAlign) {
			double currentAngle = this.swerve.getPose().getRotation().getRadians();

			double xSetpoint = Robot.isOnRedSide ?
				Constants.Field.FIELD_X_SIZE - Constants.Field.BLUE_BARGE_SCORING_X :
				Constants.Field.BLUE_BARGE_SCORING_X;
			double rotSetpoint = Math.abs(Math.PI / 2.0 - currentAngle) < Math.abs(-Math.PI / 2.0 - currentAngle) ?
				Math.PI / 2.0 :
				-Math.PI / 2.0;

			double fixBargeX = this.fixBargeTranslationInput(
				this.xPid.calculate(this.swerve.getPose().getX(), xSetpoint));
			double fixBargeRotatin = this.fixRotationInput(
				this.turnPid.calculate(currentAngle, rotSetpoint));

			this.swerve.driveRobotRelative(new ChassisSpeeds(fixBargeX, 0.0, fixBargeRotatin));
		} else {
			Pose2d pose = null;
			switch (this.driveInputs.get().alignMode) {
				case ThroughAlign:
					pose = this.swerve.getClosestThroughScoringPose();
					break;
				
				case ReefAlign:
					Optional<Map.Entry<Integer, Pose2d>> fudged = this.swerve.getClosestFudgedScoringPose();
					pose = fudged.map(Map.Entry::getValue).orElse(null);
					break;

				case AlgaeAlign:
					pose = this.swerve.getClosestAlgaeGrabPose();
					break;
			
				default:
					System.err.println("UNREACHABLE");
					break;
			}
			if (pose == null) {
				this.swerve.driveRobotRelative(this.getSpeeds());
			} else {
				this.swerve.isAligned = this.swerve.withinTolerance(pose.getTranslation());
				Pose2d swervePose = this.swerve.getPose();
				ChassisSpeeds speeds = new ChassisSpeeds(
					this.fixTranslationInput(
						this.xPid.calculate(swervePose.getX(), pose.getX())),
					this.fixTranslationInput(
						this.yPid.calculate(swervePose.getY(), pose.getY())),
					this.fixRotationInput(
						this.turnPid.calculate(swervePose.getRotation().getRadians(), pose.getRotation().getRadians())));
				this.swerve.driveRobotRelative(speeds);
			}
		}
	}

	@Override
	public void end(boolean interrupted) {
		this.swerve.stopModules();
	}

	@Override
	public boolean isFinished() {
		return false;
	}

	public double fixTranslationInput(double n) {
		n = Math.max(
			-Constants.Swerve.MAX_ALIGN_TRANSLATION_SPEED,
			Math.min(n, Constants.Swerve.MAX_ALIGN_TRANSLATION_SPEED));
		return MathUtils.unclampedDeadzone(n, 0.03);
	}

	public double fixRotationInput(double n) {
		n = Math.max(
			-Constants.Swerve.MAX_BARGE_ALIGN_ROTAITON_SPEED, 
			Math.min(n, Constants.Swerve.MAX_BARGE_ALIGN_ROTAITON_SPEED));

		return MathUtils.unclampedDeadzone(n, 0.03);
	}

	public double fixBargeTranslationInput(double n) {
		n = Math.max(
			-Constants.Swerve.MAX_BARGE_ALIGN_TRANSLATION_SPEED, 
			Math.min(n, Constants.Swerve.MAX_BARGE_ALIGN_TRANSLATION_SPEED));

		return MathUtils.unclampedDeadzone(n, 0.03);
	}

	public ChassisSpeeds getSpeeds() {
		double x = -this.driveInputs.get().leftY;
		double y = -this.driveInputs.get().leftX;
		double rot = -this.driveInputs.get().rightX;

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
