// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
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
        public static final double shootAngle = -116; // in Degrees: measured. Theoretical was 119.6 
        
        // TODO: Define a legal starting angle, or create homing sequence
        public static final double startingAngle = intakeAngle;

        // TODO: Calibrate.
        public static final double maxVelocityDegreesPerSec = 50; // in Degrees/s
        public static final double maxAccelerationDegreesPerSecSqrd = 46; // in Degrees/s
        
        // Arm gains 
        public static final int kPIDSlot = 0;
        // TODO: Calibrate.
        public static final double kP = 0;
        public static final double kMaxOutput = 0.1;

        // Arm feedforward
        private static final double _armMaxTorque = 5.2 * 9.8 * 0.45; // in Nm: Arm mass in Kg, times gravitation, times COM distance in m.
        private static final double _motorMaxTorque = 2.7 * 2 * 33.6; // in Nm: Max measured Neo Torque in Nm, times two motors, times gear ratio.
        private static final double _motorMaxVelocity = 5800 * 2 * Math.PI / 60 / 33.6; // in rad/s: Max Neo Velocity in RPM, times 2 pi radians,
                                                                                        //           devided by 60 seconds per minute, devided by gear ration.

        // TODO: Calibrate. These are theoretical values only.
        public static final double gravityFF = _armMaxTorque / _motorMaxTorque;  // in Motor%: Required torque ratio
        public static final double velocityFF = 1 / _motorMaxVelocity; // in Motor% s/rad: 1 / Max velocity
    }
}
