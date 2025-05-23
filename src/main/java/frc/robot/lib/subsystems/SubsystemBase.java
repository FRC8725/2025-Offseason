package frc.robot.lib.subsystems;

import java.io.FileWriter;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.lib.helpers.Elastic;
import frc.robot.lib.helpers.Elastic.Notification.NotificationLevel;
import frc.robot.lib.helpers.IDashboardProvider;
import frc.robot.lib.helpers.SubsystemProvider;

public abstract class SubsystemBase extends edu.wpi.first.wpilibj2.command.SubsystemBase implements IDashboardProvider, SubsystemProvider {
    private final String subsystemName;
    private boolean recordDataMode = false; 
    private FileWriter fileWriter;

    public SubsystemBase(String name, boolean recordDataMode) {
        this.subsystemName = name;
        this.recordDataMode = recordDataMode;
        this.registerDashboard();
        RobotProvider.registerSubsystems(this);
    }

    public void registerDataName(String fileName, String... strings) {
        try {
            this.fileWriter = new FileWriter("/home/lvuser/dataRecord/" + this.subsystemName + "/" + fileName, true);
        } catch (Exception e) {
            e.printStackTrace();
        }

        if (this.fileWriter == null || this.recordDataMode) return;
        try {
            String joinedData = String.join(",", strings);
            this.fileWriter.write("Time," + joinedData);
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    public void recordData(double... doubles) {
        // TODO
        if (this.fileWriter == null || this.recordDataMode) return;
        try {
            String joinedData = String.join(",", String.valueOf(doubles));
            this.fileWriter.write(Timer.getFPGATimestamp() + "," + joinedData + "\n");
            this.fileWriter.flush();
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    public void registerAlert(Boolean... booleans) {
        for (Boolean alertBoolean : booleans) {
            if (!alertBoolean) Elastic.sendNotification(
                new Elastic.Notification()
                    .withLevel(NotificationLevel.ERROR)
                    .withDisplaySeconds(5)
                    .withTitle(subsystemName + "disconnected!!")
                    .withDescription(subsystemName + "boom")
            );
        }
    }

    @Override
    public abstract void putDashboard();

    @Override
    public abstract void periodic();
}
