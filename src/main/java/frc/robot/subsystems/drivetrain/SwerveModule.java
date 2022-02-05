package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.Drivetrain.SwerveModuleConstants;
import frc.robot.motors.ControlMode;
import frc.robot.motors.DBugSparkMax;
import frc.robot.motors.PIDFGains;
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

    private double _steerSetpoint = 0;
    private double _driveSetpoint = 0;

    public SwerveModule(SwerveModuleConstants constants) {
        this._driveMotor = createSparkMax(
                constants.idDrive,
                new UnitConversions(SwerveModuleConstants.driveRatio, SwerveModuleConstants.wheelDiameterMeters),
                constants.driveGains);

        this._steerMotor = createSparkMax(
                constants.idSteering,
                new UnitConversions(SwerveModuleConstants.steeringRatio),
                constants.steeringGains);

        this._absEncoder = createCANCoder(constants.canCoderId, constants.cancoderZeroAngle);
        this.calibrateSteering();
    }

    private static DBugSparkMax createSparkMax(int id, UnitConversions conversions, PIDFGains gains) {
        DBugSparkMax sparkMax = new DBugSparkMax(id, conversions);
        sparkMax.restoreFactoryDefaults();
        sparkMax.setupPIDF(gains);
        sparkMax.setSmartCurrentLimit(40);
        sparkMax.enableVoltageCompensation(12);
        sparkMax.setIdleMode(IdleMode.kBrake);
        sparkMax.setOpenLoopRampRate(0.01);
        sparkMax.setClosedLoopRampRate(0.01);
        return sparkMax;
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
        this._steerMotor.setPosition(_absEncoder.getAbsolutePosition() / 360 / SwerveModuleConstants.steeringRatio);
    }

    public void setSteeringPercent(double percent) {
        this._steerMotor.set(ControlMode.PercentOutput, percent);
    }

    public void setDrivePercent(double percent) {
        this._driveMotor.set(ControlMode.PercentOutput, percent);
    }

    public void stop() {
        this.setDrivePercent(0);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(
                this._driveMotor.getVelocity(VelocityUnit.MetersPerSecond),
                Rotation2d.fromDegrees(this.getAngle()));
    }

    private void placeInAppropriate0To360Scope(SwerveModuleState desiredState, Rotation2d rotation) {
        double lowerBound;
        double upperBound;
        double scopeReference = rotation.getDegrees();
        double lowerOffset = scopeReference % 360;
        double newAngle = desiredState.angle.getDegrees();

        if (lowerOffset >= 0) {
            lowerBound = scopeReference - lowerOffset;
            upperBound = lowerBound + 360;
        } else {
            upperBound = scopeReference - lowerOffset;
            lowerBound = upperBound - 360;
        }

        while (newAngle < lowerBound) {
            newAngle += 360;
        }
        while (newAngle > upperBound) {
            newAngle -= 360;
        }

        if (newAngle - scopeReference > 180) {
            newAngle -= 360;
        } else if (newAngle - scopeReference < -180) {
            newAngle += 360;
        }

        desiredState.angle = Rotation2d.fromDegrees(newAngle);
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        Rotation2d currentAngle = Rotation2d.fromDegrees(_steerMotor.getPosition(PositionUnit.Degrees));
        placeInAppropriate0To360Scope(desiredState, currentAngle); // fix angle from -180-180 to absolute reference
        
        // Optimize the reference state to avoid spinning further than 90 degrees
        desiredState = SwerveModuleState.optimize(desiredState, currentAngle);
        this._driveSetpoint = desiredState.speedMetersPerSecond;

        if (desiredState.speedMetersPerSecond != 0) // Avoid steering in place
            this._steerMotor.set(ControlMode.Position, this._steerSetpoint, PositionUnit.Degrees);

        if (desiredState.speedMetersPerSecond == 0)
            this.stop();
        else
            this._driveMotor.set(ControlMode.Velocity, _driveSetpoint, VelocityUnit.MetersPerSecond);
    }

    private static final Rotation2d kHalfRotation = Rotation2d.fromDegrees(180);

    public static SwerveModuleState optimizeState(SwerveModuleState desiredState, Rotation2d currentRotation) {

        Rotation2d angleDifference = desiredState.angle.minus(currentRotation);
        if (Math.abs(angleDifference.getDegrees()) < 90)
            return new SwerveModuleState(desiredState.speedMetersPerSecond, angleDifference);

        // invert velocity, and rotate wheel 180 degrees is better performance.
        return new SwerveModuleState(-desiredState.speedMetersPerSecond,
                angleDifference.getRadians() > 0
                        ? angleDifference.minus(kHalfRotation)
                        : angleDifference.plus(kHalfRotation));
    }

    public double getAngle() {
        double pos = this._steerMotor.getPosition(PositionUnit.Rotations);
        pos = pos - Math.floor(pos);
        return pos * 360;
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

    public double getAbsSteering() {
        return this._absEncoder.getAbsolutePosition();
    }
}