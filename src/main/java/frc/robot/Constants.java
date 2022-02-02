// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.motors.PIDFGains;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
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
    public final class Joysticks{
        public static final double deadband = 0.1;
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
