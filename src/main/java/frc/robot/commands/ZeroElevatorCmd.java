package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator;

public class ZeroElevatorCmd extends Command {
	private final boolean forced = false;
	private boolean shouldZero = false;

	public ZeroElevatorCmd() {
		this.addRequirements(Elevator.getInstance());
	}

	@Override
	public void initialize() {
		if (Elevator.isZeroed && !this.forced) {
			this.shouldZero = false;
			this.cancel();
			return;
		}
		Elevator.isZeroed = false;
		Elevator.getInstance().setZeroVoltage();
		shouldZero = true;
	}

	@Override
	public void end(boolean interrupted) {
		if (!interrupted && shouldZero) {
			Elevator.getInstance().stop();
			Elevator.getInstance().setZeroPositon();
		}
	}

	@Override
	public boolean isFinished() {
		return Elevator.getInstance().statorCurrent.getValueAsDouble() > Constants.Elevator.ZERO_MIN_CURRENT;
	}
}
