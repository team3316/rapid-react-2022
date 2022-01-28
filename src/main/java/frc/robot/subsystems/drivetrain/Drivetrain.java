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
        this._modules = new SwerveModule[] {
                new SwerveModule(Constants.Drivetrain.TRModule),
                new SwerveModule(Constants.Drivetrain.TLModule),
                new SwerveModule(Constants.Drivetrain.BRModule),
                new SwerveModule(Constants.Drivetrain.BLModule)
        };

        this._pigeonTalon = new TalonSRX(Constants.Drivetrain.pigeonTalonId);
        this._pigeon = new PigeonIMU(_pigeonTalon);

        this._odometry = new SwerveDriveOdometry(Constants.Drivetrain.kinematics, getRotation2d());
    }

    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        xSpeed = -xSpeed;
        ySpeed = -ySpeed;

        fieldRelative = fieldRelative && this._pigeon.getState() == PigeonState.Ready;
        SmartDashboard.putBoolean("Field Relative", fieldRelative);

        ChassisSpeeds speeds;
        if (fieldRelative) {
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, getRotation2d());
        } else {
            speeds = new ChassisSpeeds(xSpeed, ySpeed, rot);
        }

        var moduleStates = Constants.Drivetrain.kinematics.toSwerveModuleStates(speeds);

        setDesiredStates(moduleStates);
    }

    public void setDesiredStates(SwerveModuleState[] moduleStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates,
                Constants.Drivetrain.SwerveModuleConstants.freeSpeedMetersPerSecond);

        for (int i = 0; i < this._modules.length; i++) {
            this._modules[i].setDesiredState(moduleStates[i]);
        }
    }

    public void periodic() {
        // Update the odometry in the periodic block
        this._odometry.update(getRotation2d(), this._modules[0].getState(), this._modules[1].getState(),
                this._modules[2].getState(),
                this._modules[3].getState());
    }

    public Pose2d getPose() {
        return this._odometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        this._odometry.resetPosition(pose, getRotation2d());
    }

    public void setDriveVelocity(double velocityMetersPerSecond) {
        for (SwerveModule swerveModule : this._modules) {
            swerveModule.setDriveVelocity(velocityMetersPerSecond);
        }
    }

    private double getHeading() {
        FusionStatus status = new FusionStatus();
        this._pigeon.getFusedHeading(status);

        return status.heading;
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    public void resetYaw() {
        this._pigeon.setFusedHeading(0);
    }
}