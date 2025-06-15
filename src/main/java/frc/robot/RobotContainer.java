package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;

public class RobotContainer {
	private final Swerve swerve = new Swerve();
	// private final Vision vision = new Vision();
	private final Elevator elevator = new Elevator();
	private final Arm arm = new Arm(this.elevator::getCarriageComponentPose);
	private final Intake intake = new Intake();
	private final Joysticks joysticks = new Joysticks();
	private final SuperStructure superStructure = new SuperStructure(this.joysticks::getInput);
	
	public RobotContainer() {
		// Shuffleboard.getTab("Vision").add(this.vision);
		// Shuffleboard.getTab("Elevator").add(this.elevator);
		// Shuffleboard.getTab("Arm").add(this.arm);
		// Shuffleboard.getTab("Intake").add(this.intake);
		Shuffleboard.getTab("SuperStructure").add(this.superStructure);
	}

	public Command getAutonomousCommand() {
		return null;
	}

	// ---------- Simulation ---------- //
	private final Mechanism2d mechanisms = new Mechanism2d(1.0, 2.0);
	private final MechanismRoot2d root = this.mechanisms.getRoot("root", 0.0, 0.0);

	private final MechanismLigament2d elevatorStage = this.root
		.append(new MechanismLigament2d("ElevatorStage", 0.0, 0.0, 0.0, new Color8Bit(Color.kWhite)));
	private final MechanismLigament2d armLigament = this.root
		.append(new MechanismLigament2d("Arm", 0.0, 0.0, 0.0, new Color8Bit(Color.kOrange)));

	public void updateComponentsPose() {
		this.elevatorStage.setLength(this.elevator.getHeight());
		this.armLigament.setAngle(Units.radiansToDegrees(this.arm.getPosition()));
	}
}
