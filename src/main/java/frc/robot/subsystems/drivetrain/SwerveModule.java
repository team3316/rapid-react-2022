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
                new Rotation2d().rotateBy(Rotation2d.fromDegrees(this._steerMotor.getPosition(PositionUnit.Degrees))));
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        // Optimize the reference state to avoid spinning further than 90 degrees
        SwerveModuleState state = optimize(desiredState, this._steerMotor.getPosition(PositionUnit.Degrees));

        if (state.speedMetersPerSecond != 0) // Avoid steering in place
            this._steerMotor.set(ControlMode.Position, state.angle.getDegrees(), PositionUnit.Degrees);
            
        if (state.speedMetersPerSecond == 0)
            this.stop();
        else
            this._driveMotor.set(ControlMode.Velocity, state.speedMetersPerSecond, VelocityUnit.MetersPerSecond);
    }

    public static SwerveModuleState optimize(SwerveModuleState desiredState, double currentAngle) {
        // desired angle diff in [-360, +360]
        double _angleDiff = (desiredState.angle.getDegrees() - currentAngle) % 360;

        double targetAngle = currentAngle + _angleDiff;
        double targetSpeed = desiredState.speedMetersPerSecond;

        // Q1 undershot. We expect a CW turn.
        if (_angleDiff <= -270) targetAngle += 360;
            
        // Q2 undershot. We expect a CCW turn to Q4 & reverse direction.
        // Q3. We expect a CW turn to Q1 & reverse direction.
        else if (-90 < _angleDiff && _angleDiff < -270) {
            targetAngle += 180;
            targetSpeed = -targetSpeed;
        }

        // Q2. We expect a CCW turn to Q4 & reverse direction.
        // Q3 overshot. We expect a CW turn to Q1 & reverse direction.
        else if (90 < _angleDiff && _angleDiff < 270) {
            targetAngle -= 180;
            targetSpeed = -targetSpeed;
        }
        
        // Q4 overshot. We expect a CCW turn.
        else if (_angleDiff >= 270) targetAngle -= 360;
            
        return new SwerveModuleState(targetSpeed, Rotation2d.fromDegrees(targetAngle));
    }
}