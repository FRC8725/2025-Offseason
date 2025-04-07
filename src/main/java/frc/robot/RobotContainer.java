
package frc.robot;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.ProfiledPid_TestCmd;
import frc.robot.subsystems.ProfiledPid_Test;
import frc.robot.commands.ProfiledPid_TestCmd;

public class RobotContainer {

  private final Controller controller = new Controller();

  private final ProfiledPid_Test ProfiledPid_Test = new ProfiledPid_Test();

  private final ProfiledPid_TestCmd ProfiledPid_TestCmd = new ProfiledPid_TestCmd(ProfiledPid_Test, controller);

  public RobotContainer() {
    this.ProfiledPid_Test.setDefaultCommand(this.ProfiledPid_TestCmd);
    this.configBindings();

  }

  public void configBindings() {
    this.controller.Up()
        .onTrue(this.ProfiledPid_Test.up());
    this.controller.Down()
        .onTrue(this.ProfiledPid_Test.down());
      this.controller.test()
        .onTrue(this.ProfiledPid_Test.sysIdElevatorTest()
            .alongWith(this.ProfiledPid_Test.startCommand()));
      this.controller.start()
        .onTrue(this.ProfiledPid_Test.startCommand());
      this.controller.stop()
        .onTrue(this.ProfiledPid_Test.stopCommand());
  }

  public Command getAutonomousCommand() {
    return null;
  }
}
