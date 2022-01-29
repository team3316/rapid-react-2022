package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.Drivetrain.SwerveModuleConstants;
import frc.robot.motors.ControlMode;
import frc.robot.motors.DBugSparkMax;
import frc.robot.motors.units.PositionUnit;
import frc.robot.motors.units.UnitConversions;
import frc.robot.motors.units.VelocityUnit;

/**
 * SwerveModule
 */
public class SwerveModule {

    private DBugSparkMax _driveMotor;
    private DBugSparkMax _steerMotor;

    private CANCoder _absEncoder;

    private double _steerSetpoint;
    private double _driveSetpoint;

    public SwerveModule(SwerveModuleConstants constants) {
        this._driveMotor = new DBugSparkMax(constants.idDrive,
                new UnitConversions(SwerveModuleConstants.driveRatio, SwerveModuleConstants.wheelDiameterMeters));
        this._steerMotor = new DBugSparkMax(constants.idSteering,
                new UnitConversions(SwerveModuleConstants.driveRatio));

        this._driveMotor.setupPIDF(constants.driveGains);
        this._steerMotor.setupPIDF(constants.steeringGains);

        this._driveMotor.setSmartCurrentLimit(40);
        this._driveMotor.enableVoltageCompensation(12);

        this._steerSetpoint = 0;
        this._driveSetpoint = 0;

        this._absEncoder = createCANCoder(constants.canCoderId, constants.cancoderZeroAngle);
        this.calibrateSteering();
    }

    private static CANCoder createCANCoder(int id, double zeroAngle) {

        CANCoder canCoder = new CANCoder(id);
        // Always set CANCoder relative encoder to 0 on boot
        canCoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToZero);
        // Configure the offset angle of the magnet
        canCoder.configMagnetOffset(360 - zeroAngle);

        return canCoder;
    }

    public void calibrateSteering() {
        this._steerMotor.setPosition(_absEncoder.getAbsolutePosition() / 360);
    }

    public void setSteeringPercent(double percent) {
        this._steerMotor.set(ControlMode.PercentOutput, percent);
    }

    public void setDrivePercent(double percent) {
        this._driveMotor.set(ControlMode.PercentOutput, percent);
    }

    public void stop() {
        this.setDrivePercent(0);
        // setSteeringPercent(0);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(
                this._driveMotor.getVelocity(VelocityUnit.MetersPerSecond),
                Rotation2d.fromDegrees(this.getAngle()));
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        // Optimize the reference state to avoid spinning further than 90 degrees
        SwerveModuleState state = optimizeAngle(desiredState, Rotation2d.fromDegrees(getAngle()));

        this._steerSetpoint = this._steerMotor.getPosition(PositionUnit.Degrees) + state.angle.getDegrees();
        this._driveSetpoint = state.speedMetersPerSecond;

        if (state.speedMetersPerSecond != 0) {
            this._steerMotor.set(ControlMode.Position, this._steerSetpoint, PositionUnit.Degrees);
        }
        if (state.speedMetersPerSecond == 0)
            this.setDrivePercent(0);
        else
            this._driveMotor.set(ControlMode.Velocity, _driveSetpoint, VelocityUnit.MetersPerSecond);
    }

    public static SwerveModuleState optimizeAngle(SwerveModuleState desiredState, Rotation2d currentRadian) {
        Rotation2d angle = desiredState.angle.minus(currentRadian);
        double speed = desiredState.speedMetersPerSecond;
        if (Math.abs(angle.getDegrees()) > 90) {
            speed = -speed;
            if (angle.getRadians() > 0) {
                angle = angle.minus(Rotation2d.fromDegrees(180));
            } else {
                angle = angle.plus(Rotation2d.fromDegrees(180));
            }
        }
        return new SwerveModuleState(speed, angle);
    }

    public double getAngle() {
        double pos = this._steerMotor.getPosition(PositionUnit.Rotations);
        return (pos % 1) * 360;
    }

    public double getSteeringSetpoint() {
        return this._steerSetpoint;
    }

    public double getDriveSetpoint() {
        return this._driveSetpoint;
    }

    public double getDriveVelocity() {
        return this._driveMotor.getVelocity(VelocityUnit.MetersPerSecond);
    }

    public void setDriveVelocity(double velocityMetersPerSecond) {
        this._driveMotor.set(ControlMode.Velocity, velocityMetersPerSecond);
    }

    public void resetSteeringEncoder() {
        this._steerMotor.setPosition(0);
    }

    public double getDriveCurrent() {
        return this._driveMotor.getOutputCurrent();
    }

    public void close() {
        _driveMotor.close();
        _steerMotor.close();
        _absEncoder.DestroyObject();
    }

    public double getAbsSteering() {
        return this._absEncoder.getAbsolutePosition();
    }
}
