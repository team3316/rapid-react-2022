package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.Drivetrain.SwerveModuleConstants;
import frc.robot.motors.ControlMode;
import frc.robot.motors.DBugSparkMax;
import frc.robot.motors.IDBugMotor;
import frc.robot.motors.PositionUnit;
import frc.robot.motors.VelocityUnit;


/**
 * SwerveModule
 */
public class SwerveModule {

    private IDBugMotor _driveMotor;
    private IDBugMotor _steerMotor;

    private CANCoder _absEncoder;

    private double _steerSetpoint;
    private double _driveSetpoint;

    public SwerveModule(SwerveModuleConstants constants) {
        _driveMotor = new DBugSparkMax(constants.idDrive);
        _steerMotor = new DBugSparkMax(constants.idSteering);

        _driveMotor.setRotationFactors(PositionUnit.Meters, VelocityUnit.MetersPerSecond, SwerveModuleConstants.driveRatio, SwerveModuleConstants.wheelDiameterMeters, 3316);
        _steerMotor.setRotationFactors(PositionUnit.Rotations, VelocityUnit.RPM, SwerveModuleConstants.steeringRatio, 3316, 3316);

        _driveMotor.setupPIDF(constants.driveGains);
        _steerMotor.setupPIDF(constants.steeringGains);

        ((CANSparkMax) _driveMotor).setSmartCurrentLimit(40);
        ((CANSparkMax) _driveMotor).enableVoltageCompensation(12);

        _steerSetpoint = 0;
        _driveSetpoint = 0;

        _absEncoder = configCANCoder(constants.canCoderId, constants.cancoderZeroAngle);
        calibrateSteering();
    }

    
    private static CANCoder configCANCoder(int id, double zeroAngle) {
        
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

    public void setDriveSteering(double percent) {
        this._steerMotor.set(ControlMode.PercentOutput,percent);
    }
    public void setDriveDrive(double percent) {
        this._driveMotor.set(ControlMode.PercentOutput,percent);
    }

    public void stop() {
        setDriveDrive(0);
        // setDriveSteering(0);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(
                _driveMotor.getVelocity(),
                new Rotation2d(getAngle()));
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        // Optimize the reference state to avoid spinning further than 90 degrees
        SwerveModuleState state = optimizeAngle(desiredState, Rotation2d.fromDegrees(getAngle()));

        _steerSetpoint = _steerMotor.getPosition() + state.angle.getDegrees() / 360;
        System.out.println(state.angle.getDegrees() + "  " + _steerMotor.getPosition() + " " + desiredState.angle.getDegrees()
        );
        _driveSetpoint = state.speedMetersPerSecond;

        if (state.speedMetersPerSecond != 0) {
            _steerMotor.set(ControlMode.Position, _steerSetpoint);
        }
        if (state.speedMetersPerSecond == 0)
            _driveMotor.set(ControlMode.PercentOutput,0);
        else
            _driveMotor.set(ControlMode.Velocity,_driveSetpoint);
    }

    
    public static SwerveModuleState optimizeAngle(SwerveModuleState desiredState, Rotation2d currentRadian) {
        Rotation2d angle = desiredState.angle.minus(currentRadian);
        double speed = desiredState.speedMetersPerSecond;
        if(Math.abs(angle.getDegrees()) > 90) {
            speed = -speed;
            if (angle.getRadians() > 0) {
                angle = angle.minus(Rotation2d.fromDegrees(180));
            } else {
                angle = angle.plus(Rotation2d.fromDegrees(180));
            }
        }
        return new SwerveModuleState(speed,angle);
    }
    public double getAngle() {
        double pos = _steerMotor.getPosition();
        pos = pos - Math.floor(pos);
        return pos * 360;
    }

    public double getSteeringSetpoint() {
        return _steerSetpoint;
    }

    public double getDriveSetpoint() {
        return _driveSetpoint;
    }

    public double getDriveVelocity() {
        return _driveMotor.getVelocity();
    }

    public void setDriveVelocity(double velocityMetersPerSecond) {
        _driveMotor.set(ControlMode.Velocity, velocityMetersPerSecond);
    }

    public void resetSteeringEncoder() {
        _steerMotor.setPosition(0);
    }

    public double getDriveCurrent() {
        return ((CANSparkMax) _driveMotor).getOutputCurrent();
    }
}
