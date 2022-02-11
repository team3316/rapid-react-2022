package frc.robot.humanIO;

<<<<<<< HEAD
=======
import edu.wpi.first.wpilibj.XboxController.Button;
>>>>>>> e8d6be1 (changed to PS5 controller)
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants;
import frc.robot.humanIO.PS5Controller.Button;

public class Joysticks {

<<<<<<< HEAD
    private PS5Controller _controller;

    public Joysticks() {
        this._controller = new PS5Controller(0);
    }

    private static double calculateDeadband(double value, double other) {
        return Math.abs(value) > Constants.Joysticks.deadband
                || Math.abs(other) > Constants.Joysticks.deadband
                        ? value
                        : 0;
    }

    public double getSteerX() {
        double leftAxis = (this._controller.getL2Axis() + 1) / 2;
        double rightAxis = (this._controller.getR2Axis() + 1) / 2;

        return leftAxis > rightAxis ? squareInputs(leftAxis) : squareInputs(-rightAxis);
    }

    private double squareInputs(double input) {
        return Math.copySign(input * input, input);
=======
    private PS5Controller _Controller;

    public Joysticks(){
        this._Controller = new PS5Controller(Constants.Joysticks.port);
>>>>>>> e8d6be1 (changed to PS5 controller)
    }

    public double getDriveY() {
        return squareInputs(calculateDeadband(-_controller.getLeftY(), -_controller.getLeftX()));
    }

    public double getDriveX() {
        return squareInputs(calculateDeadband(_controller.getLeftX(), -_controller.getLeftY()));
    }

    public JoystickButton getButton(Button button) {
        return new JoystickButton(_controller, button.value);
    }

}
