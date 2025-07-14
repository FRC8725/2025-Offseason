package frc.robot;

import com.fasterxml.jackson.databind.node.ArrayNode;

import au.grapplerobotics.CanBridge;
import choreo.Choreo;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.AutoRunnerCmd;
import frc.robot.commands.DriveCmd;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.Swerve;

public class Robot extends TimedRobot {
	private Command autonomousCommand = new InstantCommand();
	private final RobotContainer robotContainer;
	private boolean wasCoastModeEnabled = false;
	private boolean wasEnabled = false;
	private boolean wasEnabledThenDisabled = false;

	private final DigitalInput input = new DigitalInput(1);
	
	public static final boolean isRedAlliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red;
	public static final boolean isOnRedSide = (Swerve.getInstance() == null ? 0.0 : Swerve.getInstance().getPose().getX())  > (Constants.Field.FIELD_X_SIZE / 2.0);
	public static double tick = 0.0;

	// ---------- Simualtion ---------- //
	private final StructArrayPublisher<Pose2d> trajectoryPublisher = NetworkTableInstance.getDefault()
		.getStructArrayTopic("TrajectoryPose", Pose2d.struct).publish();

	// ---------- Autos ---------- //
	private final SendableChooser<Trajectory<SwerveSample>> chooser = new SendableChooser<>();
	private Trajectory<SwerveSample> trajectory = null;
	private boolean didRunAuto = false;

	public Robot() {
		super(0.02);
		this.robotContainer = new RobotContainer();
		
		this.chooser.setDefaultOption("Null", null);
		for (String trajectoryName : Choreo.availableTrajectories()) {
			if (!trajectoryName.equals("VariablePoses")) {
				this.chooser.addOption(trajectoryName, Choreo.<SwerveSample>loadTrajectory(trajectoryName).get());
			}
		}

		this.chooser.onChange(t -> {
			this.trajectory = t;
			if (this.trajectory == null) return;
			this.trajectoryPublisher.accept(this.trajectory.getPoses());
			this.initializeAutonomousCommand();
		});

		SmartDashboard.putData("Chooser", this.chooser);
	}

	@Override
	public void robotPeriodic() {
		tick++;
		CommandScheduler.getInstance().run();
	}

	// ---------- Disabled ---------- //
	@Override
	public void disabledInit() {
		if (this.wasEnabled) this.wasEnabledThenDisabled = true;
		this.wasEnabled = false;
	}

	@Override
	public void disabledPeriodic() {
		boolean pressed = !this.input.get();
		if (!this.wasCoastModeEnabled && pressed) {
			Elevator.getInstance().setCoastMode(true);
			Arm.getInstance().setCoastEnabled(true);
			this.wasCoastModeEnabled = true;
		} else if (this.wasCoastModeEnabled && !pressed) {
			Elevator.getInstance().setCoastMode(false);
			Arm.getInstance().setCoastEnabled(false);
			this.wasCoastModeEnabled = false;
		}
	}

	@Override
	public void disabledExit() {
		Elevator.getInstance().setCoastMode(false);
		Arm.getInstance().setCoastEnabled(false);
		this.wasEnabled = true;
		this.wasEnabledThenDisabled = false;
	}

	// ---------- Autonomous ---------- //
	private void initializeAutonomousCommand() {
		if (this.trajectory == null) return;
		this.autonomousCommand = SuperStructure.getInstance().makeZeroAllSubsystemsCommand()
			.andThen(new AutoRunnerCmd(SuperStructure.getInstance(), this.trajectory, Swerve.getInstance()));
	}

	@Override
	public void autonomousInit() {
		Arm.hasObject = true;
		Arm.getInstance().autoTimer.reset();
		Arm.getInstance().autoTimer.start();
		this.didRunAuto = true;
		this.autonomousCommand.schedule();
	}

	@Override
	public void autonomousPeriodic() {}

	@Override
	public void autonomousExit() {
		this.autonomousCommand.cancel();
	}

	// ---------- Teleoperate ---------- //
	@Override
	public void teleopInit() {
		if (!this.didRunAuto) Swerve.getInstance().resetYaw(isRedAlliance ? Math.PI : 0.0);
		this.robotContainer.teleInit();
	}

	@Override
	public void teleopExit() {
		Swerve.getInstance().removeDefaultCommand();
		SuperStructure.getInstance().removeDefaultCommand();
	}

	// ---------- Test ---------- //
	@Override
	public void testInit() {
		CommandScheduler.getInstance().cancelAll();
		SuperStructure.getInstance().makeZeroAllSubsystemsCommand().schedule();
		Swerve.getInstance().resetYaw(0.0);
	}

	// ---------- Simulation ---------- //
	@Override
	public void simulationInit() {}

	@Override
	public void simulationPeriodic() {
		this.robotContainer.updateSimulation();
	}
}
