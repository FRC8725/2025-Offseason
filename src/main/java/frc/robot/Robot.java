package frc.robot;

import choreo.Choreo;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Arm;

public class Robot extends TimedRobot {
	private Command autonomousCommand;
	private final RobotContainer robotContainer;
	
	public static final boolean isRedAlliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red;
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
	public void disabledPeriodic() {}

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
