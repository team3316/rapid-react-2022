// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.trigger;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.motors.BetterServo;

public class Trigger extends SubsystemBase {

    private BetterServo _servoLeft, _servoRight;

    public Trigger() {
        this._servoLeft = new BetterServo(Constants.Trigger.Left.channel);
        this._servoRight = new BetterServo(Constants.Trigger.Right.channel);

        setLeftAngle(Constants.Trigger.Left.inAngle);
        setRightAngle(Constants.Trigger.Right.inAngle);
    }

    public void setLeftAngle(double angle) {
        this._servoLeft.setAngle(angle);
    }

    public void setRightAngle(double angle) {
        this._servoRight.setAngle(angle);
    }

    public double getLeftAngle() {
        return this._servoLeft.getAngle();
    }

    public double getRightAngle() {
        return this._servoRight.getAngle();
    }

    @Override
    public void periodic() {
        // updateSDB();
    }

    // private void updateSDB() {
    // SmartDashboard.putBoolean(
    // "Left Trigger Out",
    // Math.abs(getLeftAngle() - Constants.Trigger.Left.outAngle) <= 0.01);

    // SmartDashboard.putBoolean(
    // "Right Trigger Out",
    // Math.abs(getRightAngle() - Constants.Trigger.Right.outAngle) <= 0.01);
    // }
}
