package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.lib.helpers.DashboardHelper;
import frc.robot.lib.subsystems.RobotProvider;

public class Robot extends TimedRobot {
	private Command autonomousCommand;
	private final RobotContainer robotContainer;

	public Robot() {
		super(0.01);
		DashboardHelper.enableRegistration();
		this.robotContainer = new RobotContainer();
		DashboardHelper.disableRegistration();
	}

	@Override
	public void robotPeriodic() {
		CommandScheduler.getInstance().run();
		// RobotProvider.periodic();
		DashboardHelper.putAllRegistries();
	}

	@Override
	public void disabledInit() {}

	@Override
	public void disabledPeriodic() {}

	@Override
	public void autonomousInit() {
		this.autonomousCommand = this.robotContainer.getAutonomousCommand();

		if (this.autonomousCommand != null) {
			this.autonomousCommand.schedule();
		}
	}

	@Override
	public void autonomousPeriodic() {}

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
	public void simulationPeriodic() {}
}
