package frc.robot.lib.math;

import java.util.ArrayList;
import java.util.Arrays;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

import frc.robot.Constants;

public class TestPoint {
    public static void main(String[] args) {
        ArrayList<Pose2d> poseArray = new ArrayList<>();
        // Blue scoring poses
        for (int angle = 0; angle < 360; angle += 60) {
            for (int direction : new int[]{1, -1}) {
                for (int side : new int[]{1, -1}) {
                    Pose2d pose = new Pose2d(Constants.Field.BLUE_REEF_CENTER, Rotation2d.kZero)
                        .plus(new Transform2d(
                            new Translation2d(
                                -Constants.Field.ROBOT_REEF_CENTER_DISTANCE,
                                new Rotation2d(Units.degreesToRadians(angle))),
                            Rotation2d.kZero))
                        .plus(new Transform2d(
                            new Translation2d(
                                direction * Constants.Field.REEF_BRANCE_OFFSET_DISTANCE,
                                new Rotation2d(Units.degreesToRadians(angle + 90.0))),
                            new Rotation2d(Units.degreesToRadians(angle))))
                        .plus(new Transform2d(
                            0.0, side * Constants.Arm.CORAL_CENTER_OFFSET, new Rotation2d(-side * Math.PI / 2.0)));
                    System.out.println(pose.getX() + " " + pose.getY() + " " + pose.getRotation().getRadians());
                    poseArray.add(pose);
                }
            }
        }
    }
}
