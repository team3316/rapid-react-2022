package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;
import frc.robot.Constants.Drivetrain.SwerveModuleConstants;
import frc.robot.motors.PIDFGains;


/**
 * SwerveModule
 */
public class SwerveModule {

    private CANSparkMax _driveSparkMax;
    private SparkMaxPIDController _drivePID;
    private RelativeEncoder _driveEncoder;
    private CANSparkMax _steeringSparkMax;
    private SparkMaxPIDController _steeringPID;
    private RelativeEncoder _steeringEncoder;

    private CANCoder _absEncoder;

    private double _steerSetpoint;
    private double _driveSetpoint;

    public SwerveModule(SwerveModuleConstants constants) {
        _driveSparkMax = configSparkMax(constants.idDrive, _drivePID, _driveEncoder, constants.driveGains);
        _steeringSparkMax = configSparkMax(constants.idSteering, _steeringPID, _steeringEncoder, constants.steeringGains);

        _drivePID = _driveSparkMax.getPIDController();
        _driveEncoder = _driveSparkMax.getEncoder();

        _steeringPID = _steeringSparkMax.getPIDController();
        _steeringEncoder = _steeringSparkMax.getEncoder();

        setPIDGains(_drivePID, constants.driveGains);
        setPIDGains(_steeringPID, constants.steeringGains);

        _steerSetpoint = 0;

        _absEncoder = configCANCoder(constants.canCoderId, constants.cancoderZeroAngle);
        calibrateSteering();
    }

    private static CANSparkMax configSparkMax(int id, SparkMaxPIDController pidController, RelativeEncoder encoder, PIDFGains gains) {
        CANSparkMax sparkMax = new CANSparkMax(id, MotorType.kBrushless);
        sparkMax.setInverted(false);
        sparkMax.setSmartCurrentLimit(40);
        sparkMax.enableVoltageCompensation(12);

        return sparkMax;
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
        this._steeringEncoder.setPosition(_absEncoder.getAbsolutePosition() / 360 / Constants.Drivetrain.SwerveModuleConstants.steeringRatio);
    }

    private static void setPIDGains(SparkMaxPIDController pidController, PIDFGains gains) {
        pidController.setI(gains.kI);
        pidController.setP(gains.kP);
        pidController.setD(gains.kD);
        pidController.setFF(gains.kF);
        pidController.setIZone(gains.iZone);
        pidController.setOutputRange(-1.0,1.0);
    }

    public void setDriveSteering(double percent) {
        this._steeringSparkMax.set(percent);
    }
    public void setDriveDrive(double voltage) {
        this._driveSparkMax.setVoltage(voltage);
    }

    public void stop() {
        setDriveDrive(0);
        // setDriveSteering(0);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(
                _driveEncoder.getVelocity() / 60 * Constants.Drivetrain.SwerveModuleConstants.driveDPRMeters,
                new Rotation2d(getAngle()));
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        // Optimize the reference state to avoid spinning further than 90 degrees
        SwerveModuleState state = optimizeAngle(desiredState, Rotation2d.fromDegrees(getAngle()));

        _steerSetpoint = addDeltaFromZeroToEncoder(state.angle.getDegrees());
        _driveSetpoint = driveVelocityToRPM(state.speedMetersPerSecond);

        if (state.speedMetersPerSecond != 0) {
            _steeringPID.setReference(_steerSetpoint, ControlType.kPosition);
        }
        // _driveSparkMax.set(1 * Math.signum(state.speedMetersPerSecond));
        if (state.speedMetersPerSecond == 0)
            _driveSparkMax.set(0);
        else
            _drivePID.setReference(_driveSetpoint, ControlType.kVelocity);
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

    public double addDeltaFromZeroToEncoder(double angle) {
        double pos = getAbsSteeringPos();
        return (pos + (angle / 360)) / Constants.Drivetrain.SwerveModuleConstants.steeringRatio;
    }

    private double driveVelocityToRPM(double velocity) {
        // divide by distance per revolution, multiply by a minute to get RPM
        return velocity / (Constants.Drivetrain.SwerveModuleConstants.driveDPRMeters) * 60;
    }

    public double getAbsSteeringPos() {
        return _steeringEncoder.getPosition() * Constants.Drivetrain.SwerveModuleConstants.steeringRatio;
    }

    public double getAngle() {
        double pos = getAbsSteeringPos();
        pos = pos - Math.floor(pos);
        return pos * 360;
    }

    public double getDrivePercent() {
        return _driveSparkMax.get();
    }

    public double getSteeringSetpoint() {
        return _steerSetpoint;
    }

    public double getDriveSetpoint() {
        return _driveSetpoint;
    }

    public double getDriveRPM() {
        return _driveEncoder.getVelocity();
    }

    public void setDriveRPM(double RPM) {
        _drivePID.setReference(RPM,ControlType.kVelocity);
    }

    public void resetSteeringEncoder() {
        _steeringEncoder.setPosition(0);
    }

    public double getDriveCurrent() {
        return _driveSparkMax.getOutputCurrent();
    }
}
