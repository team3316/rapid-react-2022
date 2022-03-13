package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.motors.PIDFGains;

/**
 * Constants
 */
public final class Constants {
    public static final class ArmConstants {
        // Arm motors
        public static final int leaderCANID = 16;
        public static final int followerCANID = 17;

        public static final boolean motorInverted = false;

        public static final double motorToArmConversionFactor = 360 / 33.6; // in Degrees: 360 degrees / gear
                                                                            // reduction
        public static final double keepPrecent = -0.1;

        // Arm motion
        public static final double overshootDelta = 1;
        public static final double intakeAngle = 36 + overshootDelta; // in Degrees: measured. Theoretical was -37.6
        public static final double shootAngle = -118; // in Degrees: measured. Theoretical was 119.6

        public static final double startingAngle = -90;

        // motion profile.
        public static final double movementTime = 1.5; // in secs.
        public static final double maxVelocityDegreesPerSec = 180 * 2 / movementTime; // in Degrees/s
        public static final double maxAccelerationDegreesPerSecSqrd = maxVelocityDegreesPerSec / (movementTime / 2); // in
                                                                                                                     // Degrees/s^2
        public static final Constraints trapezoidConstraints = new Constraints(maxVelocityDegreesPerSec,
                maxAccelerationDegreesPerSecSqrd);

        // Arm gains
        public static final int kPIDSlot = 0;
        public static final double kP = 0.03;
        public static final double kMaxOutput = 0.4;
        public static final PIDFGains gains = new PIDFGains(kP, 0, 0, 0, kMaxOutput);

        // Arm feedforward
        public static final double gravityFF = -0.07; // in Motor%
        public static final double velocityFF = 0.000965; // in Motor% s/degs
    }

    public static final class Drivetrain {
        public static class SwerveModuleConstants {
            public static final double driveKp = 0.612; // in seconds per meter
            public static final double driveKd = 0; // in seconds per meter
            public static final double driveKf = 0.75 / 2.81; // percent to motor / m/s at that
                                                              // percent
            public static final double steeringKp = 0.0124; // in 1 / wheel degrees

            private static final double neoMaxSpeed = 5600;
            private static final double driveRatio = 1.0 / 8.14;
            private static final double steeringRatio = 1.0 / 12.8;
            private static final double wheelDiameterMeters = 4 * 2.54 / 100; // 4 inches in meters

            public static final double drivePositionConversionFactor = driveRatio * wheelDiameterMeters * Math.PI; // m
                                                                                                                   // /
                                                                                                                   // rotation
            public static final double driveVelocityConversionFactor = drivePositionConversionFactor / 60; // m /
                                                                                                           // (rotation
                                                                                                           // *
                                                                                                           // seconds/minute)

            public static final double steeringPositionConversionFactor = steeringRatio * 360; // degrees / rotation
            public static final double steeringVelocityConversionFactor = steeringPositionConversionFactor / 60; // degrees
                                                                                                                 // /
                                                                                                                 // (rotation
                                                                                                                 // *
                                                                                                                 // seconds/minute)

            public static final double freeSpeedMetersPerSecond = neoMaxSpeed * driveVelocityConversionFactor;

            public final Translation2d position;
            public final int idDrive;
            public final PIDFGains driveGains = new PIDFGains(driveKp, 0, driveKd, driveKf);
            public final int idSteering;
            public final PIDFGains steeringGains = new PIDFGains(steeringKp);
            public final double cancoderZeroAngle;
            public final int canCoderId;

            public SwerveModuleConstants(
                    Translation2d position,
                    int idDrive,
                    int idSteering,
                    double cancoderZeroAngle,
                    int canCoderId) {

                this.position = position;
                this.idDrive = idDrive;
                this.idSteering = idSteering;
                this.cancoderZeroAngle = cancoderZeroAngle;
                this.canCoderId = canCoderId;
            }
        }

        public static final double frontWheelDistMeters = 0.6703;
        public static final double sideWheelDistMeters = 0.5102;

        public final static double cancoderTLOffset = 10.1;
        public final static double cancoderTROffset = 234.3;
        public final static double cancoderBLOffset = 109.6;
        public final static double cancoderBROffset = 159.3;

        public static final SwerveModuleConstants TLModule = new SwerveModuleConstants(
                new Translation2d(-frontWheelDistMeters / 2, sideWheelDistMeters / 2), 3, 4,
                cancoderTLOffset, 11);

        public static final SwerveModuleConstants TRModule = new SwerveModuleConstants(
                new Translation2d(frontWheelDistMeters / 2, sideWheelDistMeters / 2), 1, 2,
                cancoderTROffset, 10);

        public static final SwerveModuleConstants BLModule = new SwerveModuleConstants(
                new Translation2d(-frontWheelDistMeters / 2, -sideWheelDistMeters / 2), 5, 6,
                cancoderBLOffset, 12);

        public static final SwerveModuleConstants BRModule = new SwerveModuleConstants(
                new Translation2d(frontWheelDistMeters / 2, -sideWheelDistMeters / 2), 7, 8,
                cancoderBROffset, 13);

        public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(TRModule.position,
                TLModule.position, BRModule.position, BLModule.position);

        public static final int pigeonId = 9;
    }

    public static final class Manipulator {
        public static final int leaderId = 14;
        public static final int followerId = 15;

        public static final int leftChannel = 8;
        public static final int rightChannel = 9;

        public static final double kP = 0.05; //
        public static final double kF = 0.75 * 1023 / 13360; // units at 75% / measured native velocity at 75%
        public static final PIDFGains gains = new PIDFGains(kP, 0, 0, kF);

        public static final double collectRPM = 2400;
        public static final double shootRPM = -4250;

        public static final double maxAccelerationSeconds = 0.25;
        public static final double kVelocityConversionFactor = 600.0 / 2048; // 100ms per minute / upr
                                                                             // (native velocity to RPM)
        public static final double kPeakOutput = 1;
    }

    public final static class Trigger {
        // 200 to give it the maximum angle
        public final static class Left {
            public final static int channel = 9;
            public final static double inAngle = 0.0;
            public final static double outAngle = 200.0;
        }

        public final static class Right {
            public final static int channel = 8;
            public final static double inAngle = 200.0;
            public final static double outAngle = 0.0;
        }
    }

    public static final class Joysticks {
        public static final double valueScalar = 1;
        public static final double deadband = 0.08;
        public static final int driverControllerPort = 0;
    }

    public final class Climber {
        public static final int leftID = 19;
        public static final int rightID = 18;
        public final static int highID = 20;

        public static final double midGearRatio = 12.0;
        public static final double highGearRatio = 27.0;

        public static final double voltageCompensation = 10.0;

        public static final double startingPosition = 0.0;
        public static final double midExtentionHeight = 0.95;
        public static final double highExtentionHeight = 0.95;

        public static final double midConversionFactor = 35e-3 * Math.PI / midGearRatio; // in m: Winch diameter is 30mm
        public static final double highConversionFactor = 35e-3 * Math.PI / highGearRatio; // in m: Winch diameter is 30mm
    }

    public static final class Autonomous {
        public static final double kMaxSpeedMetersPerSecond = 2.0;
        public static final double kMaxAccelerationMetersPerSecondSquared = 2.0;
        public static final double kMaxAngularSpeedRadiansPerSecond = 1.5;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = 5;

        public static final String defaultPath = "rotate";

        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 3;

        public static final Constraints kThetaControllerConstraints = new Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }

    public static final class LED {
        // TODO: change values
        public static final int port = 0;
        public static final int length = 0;

        public static final Color oneCargoColor = Color.kBlue;
        public static final Color bothCargoColor = Color.kBlue;
        public static final Color noneCargoColor = Color.kBlue;
        public static final Color defaultCargoColor = Color.kBlue;

        public static final Color armUpColor = Color.kBlue;

        public static final Color maxHeightClimbColor = Color.kBlue;

        public static final double endgameTime = 15.0;
        public static final Color fifteenSecColor = Color.kBlue;
        public static final Color fiveSecColor = Color.kBlue;

    }
}
