package frc.robot;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.DriveCmd;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.Swerve;

public class RobotContainer {
	private final Swerve swerve = new Swerve();
	// private final Vision vision = new Vision();
	private final Elevator elevator = new Elevator();
	private final Joysticks joysticks = new Joysticks();
	private final SuperStructure superStructure = new SuperStructure(this.joysticks::getInput);
	private final Arm arm = new Arm(
		this.joysticks.wantOffsetArmPositive,
		this.joysticks.wantOffsetArmNegative);
	private final Intake intake = new Intake(
		this.superStructure.input);

	private final CoralSim coralSim = new CoralSim(
		this.superStructure, this.swerve::getPose);
	
	public RobotContainer() {
		this.swerve.setDefaultCommand(new DriveCmd(this.swerve, this.joysticks::getDriveInput));
		
		// Shuffleboard.getTab("Vision").add(this.vision);
		Shuffleboard.getTab("Elevator").add(this.elevator);
		Shuffleboard.getTab("Arm").add(this.arm);
		Shuffleboard.getTab("Intake").add(this.intake);
		Shuffleboard.getTab("SuperStructure").add(this.superStructure);
	}

	public Command getAutonomousCommand() {
		return null;
	}

	// ---------- Simulation ---------- //
	public void updateSimulation() {
		this.elevator.simulationUpdate();
		this.intake.simulationUpdate();
		this.arm.simulationUpdate();	
		this.coralSim.simulationUpdate();	
	}
}
