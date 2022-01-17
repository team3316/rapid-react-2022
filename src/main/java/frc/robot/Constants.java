package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.motors.PIDFGains;

/**
 * Constants
 */
public class Constants {

    public static final class Drivetrain {
        public static class SwerveModuleConstants {
            public static final double freeSpeedMetersPerSecond = 3.6576;
            public static final double driveRatio = 1.0 / 8.14;
            public static final double steeringRatio = 1.0 / 12.8;
            public static final double wheelDiameterMeters = 4 * 2.54 / 100; // 4 inches in meters
            // public static final double wheelCircumferenceMeters = wheelRadiusMeters * 2 * Math.PI;
            // public static final double driveDPRMeters = wheelCircumferenceMeters * driveRatio;

            public final Translation2d position;
            public final int idDrive;
            public final PIDFGains driveGains;
            public final int idSteering;
            public final PIDFGains steeringGains;
            public final double cancoderZeroAngle;
            public final int canCoderId;

            public SwerveModuleConstants(Translation2d position, int idDrive, int idSteering, double cancoderZeroAngle,int canCoderId) {
                this(position, idDrive, idSteering, new PIDFGains(0.0002 * 60/ (driveRatio * wheelDiameterMeters * Math.PI), 0, 0.01 * 60/ (driveRatio * wheelDiameterMeters * Math.PI), 1.0/6/902.0 * 0.95 * 60 / (driveRatio * wheelDiameterMeters * Math.PI), 50, 0), new PIDFGains(0.35 / steeringRatio, 0.0, 0, 0, 1, 0), cancoderZeroAngle, canCoderId);
            }

            public SwerveModuleConstants(Translation2d position, int idDrive, int idSteering, PIDFGains driveGains, PIDFGains steeringGains, double cancoderZeroAngle, int canCoderId) {
                this.position = position;
                this.idDrive = idDrive;
                this.driveGains = driveGains;
                this.idSteering = idSteering;
                this.steeringGains = steeringGains;
                this.cancoderZeroAngle = cancoderZeroAngle;
                this.canCoderId = canCoderId;
            }
        }

        public static final SwerveModuleConstants TLModule = new SwerveModuleConstants(new Translation2d(-0.215, 0.215), 3, 4, 188.8, 11);
        public static final SwerveModuleConstants TRModule = new SwerveModuleConstants(new Translation2d(0.215, 0.215), 1, 2, 122.3, 10);
        public static final SwerveModuleConstants BLModule = new SwerveModuleConstants(new Translation2d(-0.215, -0.215), 5, 6, 290.4, 12);
        public static final SwerveModuleConstants BRModule = new SwerveModuleConstants(new Translation2d(0.215, -0.215), 7, 8, 143.0, 13);

        public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(TRModule.position,
                TLModule.position, BRModule.position, BLModule.position);

        public static final int pigeonTalonId = 9;
    }
    public static final class Joysticks {
        public static final double valueScalar = 1;
        public static final double deadband = 0.1;
        public static final int driverControllerPort = 0;
    }

    public static final class Autonomous {
        // TODO: Calibrate
        public static final double kMaxSpeedMetersPerSecond = 3.5;
        public static final double kMaxAccelerationMetersPerSecondSquared = 12.0;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

        public static final double kPXController = 0.3;
        public static final double kPYController = 0.3;
        public static final double kPThetaController = 1;

        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }
}
