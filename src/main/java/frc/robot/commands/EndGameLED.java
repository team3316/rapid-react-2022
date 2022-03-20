// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.LED.LEDType;
import frc.robot.Constants.LED.RobotColorState;
import frc.robot.subsystems.LED;
import frc.robot.utils.LatchedBoolean;

public class EndGameLED extends CommandBase {

    private LED _led;

    private double _onInterval = 2.0;
    private final double _offInterval = 0.5;

    private LatchedBoolean _endGame;

    public EndGameLED(LED led) {
        this._led = led;
        addRequirements(this._led);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        this._led.setType(LEDType.SOLID);
        this._led.setRobotColor(RobotColorState.COLLECT);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double matchTime = DriverStation.getMatchTime();
        if (this._endGame.update(matchTime < 20.0 && matchTime > 0.0)){
            this._led.setType(LEDType.BLINK);
            if(this._led.getOn())
                this._led.setInterval(_onInterval - 0.1);
            else
                this._led.setInterval(_offInterval);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        this._led.setType(LEDType.RAINBOW);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return DriverStation.isDisabled();
    }
}
