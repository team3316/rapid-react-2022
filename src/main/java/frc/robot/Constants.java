package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.motors.PIDFGains;
import frc.robot.motors.units.UnitConversions;
import frc.robot.motors.units.VelocityUnit;

/**
 * Constants
 */
public class Constants {

    public static final class Drivetrain {
        public static class SwerveModuleConstants {
            private static final double neoMaxSpeed = 5600;
            public static final double driveRatio = 1.0 / 8.14;
            public static final double steeringRatio = 1.0 / 12.8;
            public static final double wheelDiameterMeters = 4 * 2.54 / 100; // 4 inches in meters
            public static final double freeSpeedMetersPerSecond = neoMaxSpeed *
                    new UnitConversions(driveRatio, wheelDiameterMeters)
                            .getRPMModifier(VelocityUnit.MetersPerSecond); // 3.6598

            public final Translation2d position;
            public final int idDrive;
            public final PIDFGains driveGains;
            public final int idSteering;
            public final PIDFGains steeringGains;
            public final double cancoderZeroAngle;
            public final int canCoderId;

            public SwerveModuleConstants(
                    Translation2d position,
                    int idDrive,
                    int idSteering,
                    double cancoderZeroAngle,
                    int canCoderId) {
                this(
                        position,
                        idDrive,
                        idSteering,
                        new PIDFGains(0.0002, 0, 0.01, 0.0001848, 0, 0),
                        new PIDFGains(0.35, 0.0, 0, 0, 0, 0),
                        cancoderZeroAngle,
                        canCoderId);
            }

            public SwerveModuleConstants(
                    Translation2d position,
                    int idDrive,
                    int idSteering,
                    PIDFGains driveGains,
                    PIDFGains steeringGains,
                    double cancoderZeroAngle,
                    int canCoderId) {

                this.position = position;
                this.idDrive = idDrive;
                this.driveGains = driveGains;
                this.idSteering = idSteering;
                this.steeringGains = steeringGains;
                this.cancoderZeroAngle = cancoderZeroAngle;
                this.canCoderId = canCoderId;
            }
        }

        public static final double frontWheelDistMeters = 0.6703;
        public static final double sideWheelDistMeters = 0.5102;

        public static final SwerveModuleConstants TLModule = new SwerveModuleConstants(
                new Translation2d(-frontWheelDistMeters / 2, sideWheelDistMeters / 2), 3, 4, 275.8, 11);

        public static final SwerveModuleConstants TRModule = new SwerveModuleConstants(
                new Translation2d(frontWheelDistMeters / 2, sideWheelDistMeters / 2), 1, 2, 289.6, 10);

        public static final SwerveModuleConstants BLModule = new SwerveModuleConstants(
                new Translation2d(-frontWheelDistMeters / 2, -sideWheelDistMeters / 2), 5, 6, 54.3, 12);

        public static final SwerveModuleConstants BRModule = new SwerveModuleConstants(
                new Translation2d(frontWheelDistMeters / 2, -sideWheelDistMeters / 2), 7, 8, 190.1, 13);

        public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(TRModule.position,
                TLModule.position, BRModule.position, BLModule.position);

        public static final int pigeonTalonId = 9;
    }

    public static final class Joysticks {
        public static final double valueScalar = 1;
        public static final double deadband = 0.08;
        public static final int driverControllerPort = 0;
    }

    public static final class Autonomous {
        // TODO: Calibrate
        public static final double kMaxSpeedMetersPerSecond = 3.0;
        public static final double kMaxAccelerationMetersPerSecondSquared = 8.0;
        public static final double kMaxAngularSpeedRadiansPerSecond = 11.5;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = 20;

        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 6;

        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }
    public static final class Manipulator {
        public static final int leaderId = 14;
        public static final int followerId = 15;
        public static final double kP = 0.1;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kF = 0.75 * 1023 / 14750;
        public static final double tolerance = 0;
        public static final double iZone = 0;
        public static final int leftSwitchId = 0;
        public static final int rightSwitchId = 1;
        public static final double collectRPM = 2400;
        public static final double shootRPM = -4250;
        public static final PIDFGains gains = new PIDFGains(kP, kI, kD, kF, tolerance, iZone);
        public static final int accTime = 1;
    }
    
    public final class Trigger{
        // TODO change to real value
        public final static int CHANNEL_LEFT = 8;
        public final static int CHANNEL_RIGHT = 9;
        public final static double GEAR_RATIO = 1.0;

        public final class TriggerState {
            public final static double IN_ANGLE_LEFT = 0.0;
            public final static double IN_ANGLE_RIGHT = 90.0;
            public final static double OUT_ANGLE_LEFT = 55.0;
            public final static double OUT_ANGLE_RIGHT = 0.0;
        }
    }
}
