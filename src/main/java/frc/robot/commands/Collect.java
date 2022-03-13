// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LED;
import frc.robot.Constants.LED.RobotColorState;
import frc.robot.subsystems.manipulator.Manipulator;
import frc.robot.subsystems.manipulator.Manipulator.ManipulatorState;
import frc.robot.utils.LatchedBoolean;

public class Collect extends CommandBase {

    private Manipulator _manipulator;

    private LatchedBoolean _cargoState;

    private LED _led;

    public Collect(Manipulator manipulator, LED led) {

        this._manipulator = manipulator;
        addRequirements(this._manipulator);

        this._led = led;
        this._cargoState = new LatchedBoolean();
    }

    public Collect(Manipulator manipulator) {

        this._manipulator = manipulator;
        addRequirements(this._manipulator);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        this._manipulator.setState(ManipulatorState.COLLECT);
        this._led.setLED(RobotColorState.COLLECT);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (_cargoState.update(this._manipulator.getCargoState().hasOne())) {
            this._led.setLED(RobotColorState.ONE_CARGO);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        this._manipulator.setState(ManipulatorState.OFF);
        if (interrupted) {
            this._led.setLED(RobotColorState.DEFAULT);
        } else {
            this._led.setLED(RobotColorState.TWO_CARGO);
        }
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return this._manipulator.getCargoState().hasBoth();
    }
}
