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
        //IDs
        public static final int leaderSMID = 0;
        public static final int followerSMID = 1;

        //Trapezoid Profile gains
        public static final double maxTorqueUsed = 23; //in Nm
        public static final double maxNeoTorque = 2.6;
        public static final double maxNeoRPM = 5700;
        public static final double gearRatioNeoToArm = 32;
        public static final double velFactor = 0.9;
        public static final double maxVelocityRadPerSec = velFactor*(2*maxNeoTorque-maxTorqueUsed/gearRatioNeoToArm)*(maxNeoRPM/maxNeoTorque); 
        public static final double maxAccelerationRadPerSecSqrd = 2; 

        //Arm gains
        public static final double startingRad = 2; 
        public static final PIDFGains armPID = new PIDFGains(1, 2, 3, 4, 5, 6);
        public static final double staticFF = 1/6/900;
        public static final double gravityFF = 1/6/900;
        public static final double velocityFF = 1/6/900;
        public static final double accelerationFF = 1/6/900;
        public static final double intakeAngle = 0; //to be changed
        public static final double shootAngle = 10; //to be changed
    }
}
