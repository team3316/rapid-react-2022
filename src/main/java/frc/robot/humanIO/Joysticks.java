// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.humanIO;

import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants;

/** Add your docs here. */
public class Joysticks {

    private PS5Controller _Controller;

    public Joysticks(){
        this._Controller = new PS5Controller(Constants.Joysticks.port);
    }

    public JoystickButton getButton(Button button){
        return new JoystickButton(_Controller, button.value);
    }
}
