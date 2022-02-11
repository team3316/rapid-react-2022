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
    public final class Climber {
        //TODO change to real values
        public static final int deviceNumberLeft = 0; 
        public static final int deviceNumberRight = 1;
        public static final double gearRatio = 12.0;

        public static final boolean invert = true;

        public static final double kP = 0.0;
        public static final double kF = 0.0;
        public static final double conversionFactor = 30e-3 * Math.PI / gearRatio;  // in m: Winch diameter is 30mm
    }

    public final class Joysticks {
        public static final int port = 0;
    }
}
