// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.motors.PIDFGains;

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
        //Arm IDs and info
        public static final int leaderSMID = 16;
        public static final int followerSMID = 17;
        public static final double gearRatioNeoToArm = 1/33.6; //One NEO rotation is this much arm rotations

        public static final double maxVelocityDegreesPerSec = 36; // degrees / sec
        public static final double maxAccelerationRotPerSecSqrd = 36; // degrees / sec ^ 2 
        //Arm gains 
        public static final double kP = 5e-5;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kF = 0;
        public static final double kTolerance = 0.000156;
        public static final double kIZone = 0;
        public static final PIDFGains armPID = new PIDFGains(kP, kI, kD, kF, kTolerance, kIZone);
        public static final double staticFF = 0;
        public static final double gravityFF = (0.45 * 5.2 * 9.8) / (2.7 * 2 * 33.6) * 12;  // (required torque at 90 deg) / (max torque at 12v) * 12 volts
        public static final double velocityFF = 12 / (5800 * 2 * Math.PI / 60 / 33.6); // 12 volts / (max RPM * 2 pi rads / rev / 60 seconds / gear ratio)
        public static final double accelerationFF = (0.45 * 0.45 * 5.2) / (2.7 * 2 * 33.6) * 12;
        public static final double intakeAngle = -37; // measured. Theoretical was -37.6
        public static final double shootAngle = 126; // measured. Theoretical was 119.6 
        
        public static final double startingAngle = intakeAngle; // TODO: Define a legal starting angle, or create homing sequence 
        //torque gains
        public static final double centerOfMass = 0;
        public static final double mass = 5.2;
        //piston gains
        public static final double armLength = 0;
        public static final double armHeight = 0;
        public static final double baseLength = 0;
        public static final double baseHeight = 0;
        public static final double forceOfPiston = 300; 
        public static final double closedLength = 0; 
        public static final double extendLength = 0;
    }
}
