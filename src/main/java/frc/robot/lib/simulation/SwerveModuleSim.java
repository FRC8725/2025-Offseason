package frc.robot.lib.simulation;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

public class SwerveModuleSim {
    private final DCMotor driveMotorModule = DCMotor.getFalcon500(1);
    private final DCMotor turnMotorModule = DCMotor.getFalcon500(1);
    private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0.2187, 0.0019282, 0.00021717);

    private final DCMotorSim driveSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(
            this.driveMotorModule, 0.025, Constants.Swerve.DRIVE_GEAR_RATIO),
        this.driveMotorModule);
    private final DCMotorSim turnSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(
            this.turnMotorModule, 0.025, Constants.Swerve.TURN_GEAR_RATIO),
        this.turnMotorModule);

    private final PIDController turnPid = new PIDController(0, 0, 0);

    public SwerveModuleSim() {
        this.turnPid.enableContinuousInput(-Math.PI, Math.PI);
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            this.driveSim.getAngularPositionRotations(),
            Rotation2d.fromRadians(this.turnSim.getAngularPositionRad()));
    }

    public void setDesiredState(SwerveModuleState state) {
        if (state.speedMetersPerSecond < 0.001) {
            this.stop();
            return;
        }
        state.optimize(Rotation2d.fromRadians(this.turnSim.getAngularPositionRad()));

        double goalTurnPosition = state.angle.getRadians();
        
        double driveVoltage = this.feedforward.calculate(this.driveSim.getAngularVelocityRPM() / 60.0, Units.radiansToRotations(this.driveSim.getAngularAccelerationRadPerSecSq()));
        double turnVoltage = this.turnPid.calculate(this.turnSim.getAngularPositionRad(), goalTurnPosition);

        this.driveSim.setInputVoltage(driveVoltage);
        this.turnSim.setInput(turnVoltage);
    }

    public void stop() {
        this.driveSim.setInputVoltage(0.0);
        this.turnSim.setInputVoltage(0.0);
    }
}
