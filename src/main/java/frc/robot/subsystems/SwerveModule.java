package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.SwerveConstants;
import frc.robot.lib.encoder.SwerveEncoder;
import frc.robot.lib.helpers.IDashboardProvider;
import frc.robot.lib.motors.SwerveSpark;

public class SwerveModule implements IDashboardProvider {
    private final SwerveSpark drive;
    private final SwerveSpark turn;
    private final SwerveEncoder encoder;
    private final PIDController pid;
    private final String moduleName;

    public SwerveModule(
        int driveId, int turnId, int encoderId,
        boolean driveReverse, boolean turnReverse,
        String moduleName
    ) {
        this.registerDashboard();
        this.drive = new SwerveSpark(driveId, driveReverse, true, SwerveConstants.DRIVE_GEAR_RATIO);
        this.turn = new SwerveSpark(turnId, turnReverse, false, SwerveConstants.TURN_GEAR_RATIO);
        this.encoder = new SwerveEncoder(encoderId);
        this.pid = new PIDController(40.0, 0.1, 0.1);
        this.moduleName = moduleName;

        this.pid.enableContinuousInput(-0.5, 0.5);
        this.resetEncoder();
    }

    public void resetEncoder() {
        this.drive.getEncoder().setPosition(0.0);
        this.turn.getEncoder().setPosition(this.encoder.getAbsolutePosition().getValueAsDouble());
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(
            this.drive.getVelocity(),
            this.encoder.getRotation()
        );
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            this.drive.getPosition(),
            this.encoder.getRotation()
        );
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        if (Math.abs(desiredState.speedMetersPerSecond) < 0.001) {
            this.stop();
            return;
        }
        desiredState.optimize(this.getState().angle);

        double driveVoltage = desiredState.speedMetersPerSecond / SwerveConstants.MAX_SPEED;
        double turnVoltage = this.pid.calculate(this.encoder.getRotation().getRotations(), desiredState.angle.getRotations());

        SmartDashboard.putNumber(this.moduleName + "/desiredAngle", desiredState.angle.getRotations());
        SmartDashboard.putNumber(this.moduleName + "/turnVoltage", turnVoltage);

        this.drive.setVoltage(driveVoltage);
        this.turn.setVoltage(turnVoltage);
    }

    public void stop() {
        this.drive.stopMotor();
        this.turn.stopMotor();
    }

    @Override
    public void putDashboard() {
        SmartDashboard.putNumber(this.moduleName + "/Drive Velocity", this.drive.getVelocity());
        SmartDashboard.putNumber(this.moduleName + "/Drive Position", this.drive.getPosition());
        SmartDashboard.putNumber(this.moduleName + "/Turn Position", this.encoder.getRotation().getRotations());
    }
}
