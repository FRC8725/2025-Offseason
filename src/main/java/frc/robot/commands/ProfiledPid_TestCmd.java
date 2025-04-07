package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ProfiledPid_Test;

public class ProfiledPid_TestCmd extends Command {
	private final ProfiledPid_Test ProfiledPid_Test;
	private final XboxController controller;

	public ProfiledPid_TestCmd(ProfiledPid_Test ProfiledPid_Test , XboxController controller){
		this.ProfiledPid_Test = ProfiledPid_Test;
		this.controller = controller;
		this.addRequirements(this.ProfiledPid_Test);
	}

	@Override
	public void initialize() {
	}

	@Override
	public void execute() {
		double speed = MathUtil.applyDeadband(this.controller.getLeftY(),0.05) * 8;
		this.ProfiledPid_Test.setVoltage(speed);
	}

	@Override
	public void end(boolean interrupted) {
		this.ProfiledPid_Test.stop();
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
