package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.PigeonIMU.FusionStatus;
import com.ctre.phoenix.sensors.PigeonIMU.PigeonState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Drivetrain
 */
public class Drivetrain extends SubsystemBase {

    private SwerveModule[] _modules;

    private TalonSRX _pigeonTalon;
    private PigeonIMU _pigeon;

    private SwerveDriveOdometry _odometry;

    public Drivetrain() {
        _modules = new SwerveModule[] {
            new SwerveModule(Constants.Drivetrain.TRModule),
            new SwerveModule(Constants.Drivetrain.TLModule),
            new SwerveModule(Constants.Drivetrain.BRModule),
            new SwerveModule(Constants.Drivetrain.BLModule)
        };

        _pigeonTalon = new TalonSRX(Constants.Drivetrain.pigeonTalonId);
        _pigeon = new PigeonIMU(_pigeonTalon);

        _odometry = new SwerveDriveOdometry(Constants.Drivetrain.kinematics, getRotation2d());
    }

    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        xSpeed = -xSpeed;
        ySpeed = -ySpeed;
        rot *= 2;

        SwerveModuleState[] moduleStates = Constants.Drivetrain.kinematics
                .toSwerveModuleStates(fieldRelative && _pigeon.getState() == PigeonState.Ready
                        ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, getRotation2d())
                        : new ChassisSpeeds(xSpeed, ySpeed, rot));
        setDesiredStates(moduleStates);
    }

    public void setDesiredStates(SwerveModuleState[] moduleStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates,
                Constants.Drivetrain.SwerveModuleConstants.freeSpeedMetersPerSecond);

        for (int i = 0; i < _modules.length; i++) {
            _modules[i].setDesiredState(moduleStates[i]);
        }
    }

    public void periodic() {
        // Update the odometry in the periodic block
        _odometry.update(getRotation2d(), _modules[0].getState(), _modules[1].getState(), _modules[2].getState(),
                _modules[3].getState());
                for (int i = 0; i < _modules.length; i++) {
                    SmartDashboard.putNumber("steer " + i   , _modules[i].getSteeringSetpoint());
                    SmartDashboard.putNumber("drive " + i, _modules[i].getDriveSetpoint());
                    SmartDashboard.putNumber("speed " + i, _modules[i].getDriveVelocity());
                    SmartDashboard.putNumber("angle " + i, _modules[i].getAngle());
                }
    }

    public Pose2d getPose() {
        return _odometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        _odometry.resetPosition(pose, getRotation2d());
    }

    public void setDriveVelocity(double velocityMetersPerSecond) {
        for (SwerveModule swerveModule : _modules) {
            swerveModule.setDriveVelocity(velocityMetersPerSecond);
        }
    }

    private double getHeading() {
        FusionStatus status = new FusionStatus();
        _pigeon.getFusedHeading(status);
        
        return status.heading;
    }

    private Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    public void resetYaw() {
        _pigeon.setFusedHeading(0);
    }
}