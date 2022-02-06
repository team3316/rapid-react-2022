package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import frc.robot.motors.PIDFGains;
import frc.robot.motors.units.UnitConversions;
import frc.robot.motors.units.VelocityUnit;

/**
 * Constants
 */
public final class Constants {
    public static final class ArmConstants {
        // Arm motors
        public static final int leaderCANID = 16;
        public static final int followerCANID = 17;

        public static final boolean motorInverted = false;

        public static final double motorToArmConversionFactor = 360 / 33.6; // in Degrees: 360 degrees / gear reduction

        // Arm motion
        // TODO: Calibrate.
        public static final double intakeAngle = 36; // in Degrees: measured. Theoretical was -37.6
        public static final double shootAngle = -123; // in Degrees: measured. Theoretical was 119.6 
        
        // TODO: Define a legal starting angle, or create homing sequence
        public static final double startingAngle = intakeAngle;

        // TODO: Calibrate.
        public static final double maxVelocityDegreesPerSec = 36; // in Degrees/s
        public static final double maxAccelerationDegreesPerSecSqrd = 36; // in Degrees/s
        
        // Arm gains 
        public static final int kPIDSlot = 0;
        // TODO: Calibrate.
        public static final double kP = 0.015;
        public static final double kMaxOutput = 0.25;

        // Arm feedforward
        private static final double _armMaxTorque = 5.2 * 9.8 * 0.45; // in Nm: Arm mass in Kg, times gravitation, times COM distance in m.
        private static final double _motorMaxTorque = 2.7 * 2 * 33.6; // in Nm: Max measured Neo Torque in Nm, times two motors, times gear ratio.
        private static final double _motorMaxVelocity = 5800 * 360 / 60 / 33.6; // in Degs/s: Max Neo Velocity in RPM, times 360 degrees,
                                                                                        //           devided by 60 seconds per minute, devided by gear ration.

        // TODO: Calibrate. These are theoretical values only.
        public static final double gravityFF = -0.07;  // in Motor%: Required torque ratio
        public static final double velocityFF = 0; //1/_motorMaxVelocity; // in Motor% s/degs: 1 / Max velocity
    }
    public static final class Drivetrain {
        public static class SwerveModuleConstants {
            public static final double driveKp = 0.0002; // in minutes per motor rotation
            public static final double driveKd = 0.01; // in minutes per motor rotation
            public static final double driveKf = 0.167 / 902.0; // percent to motor / RPM of motor at that percent
            public static final double steeringKp = 0.35; // in 1 / motor rotation

            private static final double neoMaxSpeed = 5600;
            public static final double driveRatio = 1.0 / 8.14;
            public static final double steeringRatio = 1.0 / 12.8;
            public static final double wheelDiameterMeters = 4 * 2.54 / 100; // 4 inches in meters
            public static final double freeSpeedMetersPerSecond = neoMaxSpeed *
                    new UnitConversions(driveRatio, wheelDiameterMeters)
                            .getRPMModifier(VelocityUnit.MetersPerSecond); // 3.6598

            public final Translation2d position;
            public final int idDrive;
            public final PIDFGains driveGains = new PIDFGains(driveKp, 0, driveKd, driveKf, 0, 0);
            public final int idSteering;
            public final PIDFGains steeringGains = new PIDFGains(steeringKp, 0.0, 0, 0, 0, 0);
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

        public final static double cancoderTLOffset = 275.8;
        public final static double cancoderTROffset = 289.6;
        public final static double cancoderBLOffset = 54.3;
        public final static double cancoderBROffset = 190.1;

        public static final SwerveModuleConstants TLModule = new SwerveModuleConstants(
                new Translation2d(-frontWheelDistMeters / 2, sideWheelDistMeters / 2), 3, 4, cancoderTLOffset, 11);

        public static final SwerveModuleConstants TRModule = new SwerveModuleConstants(
                new Translation2d(frontWheelDistMeters / 2, sideWheelDistMeters / 2), 1, 2, cancoderTROffset, 10);

        public static final SwerveModuleConstants BLModule = new SwerveModuleConstants(
                new Translation2d(-frontWheelDistMeters / 2, -sideWheelDistMeters / 2), 5, 6, cancoderBLOffset, 12);

        public static final SwerveModuleConstants BRModule = new SwerveModuleConstants(
                new Translation2d(frontWheelDistMeters / 2, -sideWheelDistMeters / 2), 7, 8, cancoderBROffset, 13);

        public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(TRModule.position,
                TLModule.position, BRModule.position, BLModule.position);

        public static final int pigeonTalonId = 9;
    }

    public static final class Joysticks {
        public static final double valueScalar = 1;
        public static final double deadband = 0.08;
        public static final int driverControllerPort = 0;
    }
}

