package frc.robot;

import frc.robot.commands.ElevatorCmd;
import frc.robot.subsystems.ElevatorSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class RobotContainer {
  private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();

  private final Controller controller = new Controller();

  private final ElevatorCmd elevatorCommand = new ElevatorCmd(elevatorSubsystem, controller);

  public RobotContainer() {
    this.elevatorSubsystem.setDefaultCommand(elevatorCommand);
    this.configureBindings();
  }

  private void configureBindings() {
    this.controller.Up()
        .onTrue(elevatorSubsystem.up());

    this.controller.Down()
        .onTrue(elevatorSubsystem.down());

    this.controller.Start()
        .onTrue(elevatorSubsystem.start());

    this.controller.Stop()
        .onTrue(elevatorSubsystem.stop());

    this.controller.Test()
        .onTrue(elevatorSubsystem.sysIDElevator()
            .alongWith(elevatorSubsystem.start()));
  }

  public Command getAutonomousCommand() {
    return null;
  }
}
