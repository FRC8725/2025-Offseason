package frc.robot.lib;

import java.io.FileWriter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.lib.helpers.Elastic;
import frc.robot.lib.helpers.Elastic.Notification.NotificationLevel;
import frc.robot.lib.helpers.IDashboardProvider;

public abstract class SubsystemBase extends edu.wpi.first.wpilibj2.command.SubsystemBase implements IDashboardProvider {
    private final SendableChooser<Boolean> debugChooser = new SendableChooser<>();
    private final String subsystemName;
    private boolean recordDataMode = false; 
    private final boolean debug = false;
    private FileWriter fileWriter;

    public SubsystemBase(String name) {
        super(name);
        this.subsystemName = name;
        this.registerDashboard();
        RobotProvider.registerSubsystems(this);
        this.putDebugChooser();
    }

    public void setRecordDataMode(boolean enable) {
        this.recordDataMode = enable;
        try {
            this.fileWriter = new FileWriter("/home/lvuser/dataRecord/" + this.subsystemName + "\n", true);
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    public void registerDataName(String... strings) {
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

    public void putDebugChooser() {
        if (!debug) return;
        this.debugChooser.setDefaultOption("True", true);
        this.debugChooser.addOption("False", false);
        SmartDashboard.putData(this.getSubsystem(), this.debugChooser);
    }

    public boolean getChooser() {
        return this.debugChooser.getSelected();
    }

    @Override
    public abstract void putDashboard();
}