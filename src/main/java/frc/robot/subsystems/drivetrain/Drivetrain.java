package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.PigeonIMU.PigeonState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Drivetrain
 */
public class Drivetrain extends SubsystemBase {

    private SwerveModule[] _modules;

    private PigeonIMU _pigeon;

    private SwerveDriveOdometry _odometry;
    private DoubleLogEntry m_logX, m_logY, m_logR;
    private int m_counter = 0;
    private static final int LOG_EVERY = 10;

    public Drivetrain() {
        this._modules = new SwerveModule[] {
                new SwerveModule(Constants.Drivetrain.TRModule),
                new SwerveModule(Constants.Drivetrain.TLModule),
                new SwerveModule(Constants.Drivetrain.BRModule),
                new SwerveModule(Constants.Drivetrain.BLModule)
        };
        _pigeon = new PigeonIMU(Constants.Drivetrain.pigeonId); // We need the talon; not anymore

        this._odometry = new SwerveDriveOdometry(Constants.Drivetrain.kinematics, getRotation2d());

        DataLog log = DataLogManager.getLog();
        m_logX = new DoubleLogEntry(log, "/drivetrain/position/x");
        m_logY = new DoubleLogEntry(log, "/drivetrain/position/y");
        m_logR = new DoubleLogEntry(log, "/drivetrain/position/rotation");
    }

    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        fieldRelative = fieldRelative && this._pigeon.getState() == PigeonState.Ready;
        SmartDashboard.putBoolean("Field Relative", fieldRelative);

        ChassisSpeeds speeds;
        if (fieldRelative) {
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, getPose().getRotation());
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
        // updateSDB();

        if (m_counter++ == LOG_EVERY) {
            Pose2d pose = _odometry.getPoseMeters();
            m_logX.append(pose.getX());
            m_logY.append(pose.getY());
            m_logR.append(pose.getRotation().getDegrees());
            m_counter = 0;
        }

    }

    public void disabledInit() {
        for (int i = 0; i < this._modules.length; i++) {
            this._modules[i].disable();
        }
    }

    @SuppressWarnings({ "unused" })
    private void updateSDB() {
        for (int i = 0; i < this._modules.length; i++) {
            SmartDashboard.putNumber("abs " + i, this._modules[i].getAbsAngle());
        }

        SmartDashboard.putNumber("rotation", getRotation2d().getRadians());
    }

    public Pose2d getPose() {
        return this._odometry.getPoseMeters();
    }

    public double getHeading() {
        return this._pigeon.getFusedHeading();
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    public void resetYaw() {
        Pose2d pose = getPose();
        this._odometry.resetPosition(new Pose2d(pose.getTranslation(), new Rotation2d()), getRotation2d());
    }

    public void resetOdometry(Pose2d pose) {
        this._odometry.resetPosition(pose, getRotation2d());
    }

    public void calibrateSteering() {
        for (SwerveModule swerveModule : _modules) {
            swerveModule.calibrateSteering();
        }
    }
}