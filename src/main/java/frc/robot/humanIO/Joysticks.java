package frc.robot.humanIO;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants;

public class Joysticks {

    private XboxController _controller;

    public Joysticks() {
        this._controller = new XboxController(0);
    }

    private static double calculateDeadband(double value) {
        return Math.abs(value) > Constants.Joysticks.deadband ? Math.copySign(value * value, value) : 0;
    }

    private static double calculateDeadband(double value, double other) {
        return Math.abs(value) > Constants.Joysticks.deadband
                || Math.abs(other) > Constants.Joysticks.deadband
                        ? value
                        : 0;
    }

    public double getSteerX() {
        double leftAxis = this._controller.getLeftTriggerAxis();
        double rightAxis = this._controller.getRightTriggerAxis();

        return rightAxis > leftAxis ? rightAxis : -leftAxis;
    }

    public double getDriveY() {
        return calculateDeadband(-_controller.getLeftY(), -_controller.getLeftX());
    }

    public double getDriveX() {
        return calculateDeadband(_controller.getLeftX(), -_controller.getLeftY());
    }

    public JoystickButton getButton(Button button) {
        return new JoystickButton(_controller, button.value);
    }

}
