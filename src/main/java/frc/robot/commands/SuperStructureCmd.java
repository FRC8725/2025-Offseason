package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Joysticks;
import frc.robot.subsystems.SuperStructure;

public class SuperStructureCmd extends Command {
	private final Joysticks joysticks;
	
	public SuperStructureCmd(Joysticks joysticks) {
		this.joysticks = joysticks;

		this.addRequirements(SuperStructure.getInstance());
	}

	@Override
	public void execute() {
		SuperStructure.getInstance().input = this.joysticks.getInput();
	}

	@Override
	public void end(boolean interrupted) {
		SuperStructure.getInstance().emptyInputs();
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
