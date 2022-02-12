// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.humanIO;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants;
import frc.robot.humanIO.PS5Controller.Button;

public class Joysticks {

    private PS5Controller _driveController;
    private PS5Controller _operatorController;

    public Joysticks() {
        this._driveController = new PS5Controller(0);
        this._operatorController = new PS5Controller(1);
    }

    private static double calculateDeadband(double value, double other) {
        return Math.abs(value) > Constants.Joysticks.deadband
                || Math.abs(other) > Constants.Joysticks.deadband
                        ? value
                        : 0;
    }

    public double getSteerX() {
        double leftAxis = (this._driveController.getL2Axis() + 1) / 2;
        double rightAxis = (this._driveController.getR2Axis() + 1) / 2;

        return leftAxis > rightAxis ? squareInputs(leftAxis) : squareInputs(-rightAxis);
    }

    private double squareInputs(double input) {
        return Math.copySign(input * input, input);
    }

    public double getDriveY() {
        return squareInputs(calculateDeadband(-this._driveController.getLeftY(), -this._driveController.getLeftX()));
    }

    public double getDriveX() {
        return squareInputs(calculateDeadband(this._driveController.getLeftX(), -this._driveController.getLeftY()));
    }

    public JoystickButton getOperatorButton(Button button) {
        return new JoystickButton(this._operatorController, button.value);
    }

    public JoystickButton getDriveButton(Button button) {
        return new JoystickButton(this._driveController, button.value);
    }

}
