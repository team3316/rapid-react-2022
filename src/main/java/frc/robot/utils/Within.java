// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

public class Within {
    public static boolean range(double value, double target, double tol){
        return Math.abs(target - value) <= tol;
    }
}
