package frc.robot.lib.simulation;

import java.net.ContentHandler;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class SwerveModuleSim {
    private final DCMotor driveMotorModule = DCMotor.getFalcon500(1);
    private final DCMotor turnMotorModule = DCMotor.getFalcon500(1);
    private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0.2199442, 2.18943902193, 0.01);

    private final DCMotorSim driveSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(
            DCMotor.getFalcon500(1), 0.025, Constants.Swerve.DRIVE_GEAR_RATIO),
        DCMotor.getFalcon500(1));
    private final DCMotorSim turnSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(
            DCMotor.getFalcon500(1), 0.025, Constants.Swerve.TURN_GEAR_RATIO),
        DCMotor.getFalcon500(1));

    private final PIDController turnPid = new PIDController(0, 0, 0);

    public SwerveModuleSim() {
        this.turnPid.enableContinuousInput(-Math.PI, Math.PI);
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            this.getDrivePosition(),
            Rotation2d.fromRadians(this.turnSim.getAngularPositionRad()));
    }

    public double getDrivePosition() {
        return this.driveSim.getAngularPositionRotations() * 2.0 * Constants.Swerve.WHEEL_RADIUS;
    }

    public double getDriveVelocity() {
        return this.driveSim.getAngularVelocityRPM() / 60.0 * 2.0 * Constants.Swerve.WHEEL_RADIUS;
    }

    public double getDriveAcceleration() {
        return Units.radiansToRotations(this.driveSim.getAngularAccelerationRadPerSecSq()) * 2.0 * Constants.Swerve.WHEEL_RADIUS;
    }

    public void setDesiredState(SwerveModuleState state) {
        if (state.speedMetersPerSecond < 0.001) {
            this.stop();
            return;
        }
        state.optimize(Rotation2d.fromRadians(this.turnSim.getAngularPositionRad()));

        double goalTurnPosition = state.angle.getRadians();
        
        double goalDriveVelocity = state.speedMetersPerSecond * Math.cos(this.turnSim.getAngularPositionRad() - goalTurnPosition);
        double driveVoltage = this.feedforward.calculate(goalDriveVelocity, this.getDriveAcceleration());
        double turnVoltage = this.turnPid.calculate(this.turnSim.getAngularPositionRad(), goalTurnPosition);

        this.driveSim.setInputVoltage(driveVoltage);
        this.driveSim.update(0.02);
        this.turnSim.setInputVoltage(turnVoltage);
        this.turnSim.update(0.02);
        

    }

    public void stop() {
        this.driveSim.setInputVoltage(0.0);
        this.turnSim.setInputVoltage(0.0);
    }
}
