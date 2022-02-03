// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.manipulator.Manipulator;

public class Shoot extends CommandBase {
    private Manipulator _manipulator;
    private final double _rpm = Constants.Manipulator.shootRPM;
    private SlewRateLimiter _limiter;

    public Shoot(Manipulator manipulator) {
        this._manipulator = manipulator;
        addRequirements(this._manipulator);
    }

    public void initialize() {
        this._limiter = new SlewRateLimiter(Math.abs(this._rpm) / Constants.Manipulator.accTime);
    }

    public void execute() {
        double output = Math.copySign(_limiter.calculate(Math.abs(this._rpm)), this._rpm);
        SmartDashboard.putNumber("limitedSetPoint", output);
        this._manipulator.set(output);
    }

    public void end(boolean interrupted) { 
        this._manipulator.set(0.0);
    }
}