// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.manipulator.Manipulator;

public class Shoot extends CommandBase {
    private Manipulator _manipulator;
    private double _rpm;
    private SlewRateLimiter _limiter;

    public Shoot(Manipulator manipulator, double rpm) {
        this._manipulator = manipulator;
        this._rpm = rpm;
        addRequirements(this._manipulator);
    }

    public void initialize() {
        this._limiter = new SlewRateLimiter(this._rpm / Constants.Manipulator.accTime);
    }

    public void execute() {
        this._manipulator.set(_limiter.calculate(this._rpm));
    }

    public void end(boolean interrupted) { 
        this._manipulator.set(0.0);
    }
}