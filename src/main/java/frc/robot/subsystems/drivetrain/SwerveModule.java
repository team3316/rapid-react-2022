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

    public void setDesiredState(SwerveModuleState desiredState) {
        // Optimize the reference state to avoid spinning further than 90 degrees
        SwerveModuleState state = optimize(desiredState, this._steerMotor.getPosition(PositionUnit.Degrees));

        this._steerSetpoint = state.angle.getDegrees();
        this._driveSetpoint = state.speedMetersPerSecond;

        if (state.speedMetersPerSecond != 0) { // Avoid steering in place
            this._steerMotor.set(ControlMode.Position, this._steerSetpoint, PositionUnit.Degrees);
        }
        if (state.speedMetersPerSecond == 0)
            this.stop();
        else
            this._driveMotor.set(ControlMode.Velocity, _driveSetpoint, VelocityUnit.MetersPerSecond);
    }

    public static SwerveModuleState optimize(SwerveModuleState desiredState, double currentAngle) {
        double _angleDiff = desiredState.angle.getDegrees() - currentAngle; // total desired angle diff
        double _reducedAngleDiff = _angleDiff % 360; // reduced desired angle diff in [-360, +360]
        int _quadrateIndex = (int) (_angleDiff / 90 % 4); // index of quadrate we desire to turn to [-3, 3]

        double targetAngle = currentAngle + _reducedAngleDiff;
        double targetSpeed = desiredState.speedMetersPerSecond;

        switch (_quadrateIndex) {
            case -3: // Q1 undershot. We expect a CW turn.
                targetAngle += 360;
                break;
            case -2: // Q2 undershot. We expect a CCW turn to Q4 & reverse direction.
            case -1: // Q3. We expect a CW turn to Q1 & reverse direction.
                targetAngle += 180;
                targetSpeed = -targetSpeed;
                break;
            case 1: // Q2. We expect a CCW turn to Q4 & reverse direction.
            case 2: // Q3 overshot. We expect a CW turn to Q1 & reverse direction.
                targetAngle -= 180;
                targetSpeed = -targetSpeed;
                break;
            case 3: // Q4 overshot. We expect a CCW turn.
                targetAngle -= 360;
                break;
            default: // Q1 or Q4. We expect a CW or CCW turn.
        }

        return new SwerveModuleState(targetSpeed, Rotation2d.fromDegrees(targetAngle));
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