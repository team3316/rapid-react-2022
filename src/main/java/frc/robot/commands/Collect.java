// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.manipulator.Manipulator;
import frc.robot.subsystems.manipulator.Manipulator.ManipulatorState;

public class Collect extends CommandBase {

    private Manipulator _manipulator;

    public Collect(Manipulator manipulator) {

        this._manipulator = manipulator;
        addRequirements(this._manipulator);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        this._manipulator.setState(ManipulatorState.COLLECT);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        this._manipulator.setState(ManipulatorState.OFF);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
