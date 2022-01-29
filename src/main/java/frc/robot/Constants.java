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
    public final class Trigger{
        // TODO change to real value
        public final static int CHANNEL_LEFT = 0;
        public final static int CHANNEL_RIGHT = 1;
        public final static double GEAR_RATIO = 1.0;

        public final class TriggerState {
            public final static double IN_ANGLE = 0.0;
            public final static double OUT_ANGLE_LEFT = 55.0;
            public final static double OUT_ANGLE_RIGHT = 55.0;
        }
    }
}
