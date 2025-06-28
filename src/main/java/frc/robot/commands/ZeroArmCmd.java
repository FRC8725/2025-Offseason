package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;

public class ZeroArmCmd extends Command {
	private final boolean forced = false;

	public ZeroArmCmd() {
		this.addRequirements(Arm.getInstance());
	}

	@Override
	public void initialize() {
		if (Arm.isZeroed && !this.forced) {
			this.cancel();
			return;
		}
		Arm.getInstance().resetRelativeFromAbsolute();
	}

	@Override
	public boolean isFinished() {
		return true;
	}
}
