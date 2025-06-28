package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;

public class ZeroIntakeCmd extends Command {
	private final boolean forced = false;
	private boolean shouldZero = false;

	public ZeroIntakeCmd() {
		this.addRequirements(Intake.getInstance());
	}

	@Override
	public void initialize() {
		if (Intake.isZeroed && !this.forced) {
			this.shouldZero = false;
			this.cancel();
			return;
		}
		Intake.isZeroed = false;
		Intake.getInstance().setZeroVoltage();
		this.shouldZero = true;
	}

	@Override
	public void end(boolean interrupted) {
		if (!interrupted && this.shouldZero) {
			Intake.getInstance().stop();
			Intake.getInstance().setZeroPosition();
		}
	}

	@Override
	public boolean isFinished() {
		return Intake.getInstance().getVelocity() < Constants.Intake.ZERO_MIN_CURRENT;
	}
}
