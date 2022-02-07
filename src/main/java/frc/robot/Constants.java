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
        // TODO check the real channel
        public static final int leftChannel = 8;
        public static final int rightChannel = 9;
        public static final double kP = 0.1; //
        public static final double kF = 0.75 * 1023 / 14750; // units at 75% / native velocity at 75%
        public static final double collectRPM = 2400;
        public static final double shootRPM = -4250;
        public static final PIDFGains gains = new PIDFGains(kP, 0, 0, kF, 0, 0);
        public static final int maxAccelerationSeconds = 1;
        public static final double kVelocityConversionFactor = 600.0 / 2048; // 100ms per minute / upr
                                                                             // (native velocity to RPM)
        public static final double kPeakOutput = 1;
    }

    public final class Joysticks {
        public static final double deadband = 0.1;
    }

    public final class Trigger {
        public final static int channelLeft = 8;
        public final static int channelRight = 9;
        // TODO check values (the triggers are in wrong place)
        public final static double inAngel = 0.0;
        public final static double outAngle = 85.0;
    }
}
