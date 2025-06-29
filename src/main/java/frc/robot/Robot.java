package frc.robot;

import au.grapplerobotics.CanBridge;
import choreo.Choreo;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Swerve;

public class Robot extends TimedRobot {
	private Command autonomousCommand;
	private final RobotContainer robotContainer;
	private final GenericEntry coastModeToggle = Shuffleboard.getTab("Autonomous")
		.add("Coast Mode", false)
		.getEntry();
	private boolean wasCoastModeEnabled = false;
	
	public static final boolean isRedAlliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red;
	public static final boolean isOnRedSide = Swerve.getInstance().getPose().getX() > (Constants.Field.FIELD_X_SIZE / 2.0);
	public static double tick = 0.0;

	// ---------- Simualtion ---------- //
	private final StructArrayPublisher<Pose2d> trajectoryPublisher = NetworkTableInstance.getDefault()
		.getStructArrayTopic("TrajectoryPose", Pose2d.struct).publish();

	// ---------- Autos ---------- //
	private final SendableChooser<Trajectory<SwerveSample>> chooser = new SendableChooser<>();
	private Trajectory<SwerveSample> trajectory;
	private boolean didRunAuto = false;

	public Robot() {
		super(0.02);
		this.robotContainer = new RobotContainer();
		CanBridge.runTCP();
		
		this.chooser.setDefaultOption("Null", null);
		for (String trajectoryName : Choreo.availableTrajectories()) {
			if (!trajectoryName.equals("VariablePoses")) {
				this.chooser.addOption(trajectoryName, Choreo.<SwerveSample>loadTrajectory(trajectoryName).get());
			}
		}

		this.chooser.onChange(t -> {
			this.trajectory = t;
			this.trajectoryPublisher.accept(this.trajectory.getPoses());
			this.initializeAutonomousCommand();
		});

		SmartDashboard.putBoolean("Coast Mode", false);
		SmartDashboard.putData("Chooser", this.chooser);
	}

	private void initializeAutonomousCommand() {
		// TODO
	}

	@Override
	public void robotPeriodic() {
		CommandScheduler.getInstance().run();
		tick++;
	}

	@Override
	public void disabledInit() {}

	@Override
	public void disabledPeriodic() {
		boolean pressed = this.coastModeToggle.getBoolean(false);
		if (!this.wasCoastModeEnabled && pressed) {
			Elevator.getInstance().setCoastMode(true);
			Arm.getInstance().setCoastEnabled(true);
			this.wasCoastModeEnabled = true;
		} else if (this.wasCoastModeEnabled && !pressed) {
			Elevator.getInstance().setCoastMode(false);
			Arm.getInstance().setCoastEnabled(false);
			this.wasCoastModeEnabled = false;
		} else {
			this.wasCoastModeEnabled = false;
			this.coastModeToggle.setBoolean(false);
		}
	}

	@Override
	public void autonomousInit() {
		Arm.hasObject = true;
		this.didRunAuto = true;
		this.autonomousCommand.schedule();
	}

	@Override
	public void autonomousExit() {
		this.autonomousCommand.cancel();
	}

	@Override
	public void teleopInit() {
		if (this.autonomousCommand != null) {
			this.autonomousCommand.cancel();
		}
	}

	@Override
	public void teleopPeriodic() {}

	@Override
	public void testInit() {
		CommandScheduler.getInstance().cancelAll();
	}

	@Override
	public void testPeriodic() {}

	@Override
	public void simulationInit() {}

	@Override
	public void simulationPeriodic() {
		this.robotContainer.updateSimulation();
	}
}
