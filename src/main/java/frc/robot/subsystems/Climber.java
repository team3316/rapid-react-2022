// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax.SoftLimitDirection;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.motors.DBugSparkMax;
import frc.robot.motors.PIDFGains;

public class Climber extends SubsystemBase {

    private DBugSparkMax _leftSparkMax, _rightSparkMax, _highClimbSparkMax;

    public Climber() {
        this._leftSparkMax = DBugSparkMax.create(Constants.Climber.leftID,
                new PIDFGains(0),
                Constants.Climber.conversionFactor,
                Constants.Climber.conversionFactor / 60,
                Constants.Climber.startingPosition);
        this._rightSparkMax = DBugSparkMax.create(Constants.Climber.rightID,
                new PIDFGains(0),
                Constants.Climber.conversionFactor,
                Constants.Climber.conversionFactor / 60,
                Constants.Climber.startingPosition);
        this._highClimbSparkMax = DBugSparkMax.create(Constants.Climber.highClimbID, new PIDFGains(0),
        Constants.Climber.conversionFactor,
        Constants.Climber.conversionFactor / 60,
        Constants.Climber.startingPosition);
                
        this._leftSparkMax.enableVoltageCompensation(Constants.Climber.voltageCompensation);
        this._rightSparkMax.enableVoltageCompensation(Constants.Climber.voltageCompensation);
        this._highClimbSparkMax.enableVoltageCompensation(Constants.Climber.voltageCompensation);

        enableSoftLimit(true);
        updateSoftLimitPosition((float) Constants.Climber.startingPosition,
                (float) Constants.Climber.midClimbExtentionHeight, (float) Constants.Climber.highClimbExtentionHeight);

        this._rightSparkMax.setInverted(true);
        this._leftSparkMax.setInverted(true);

        initSDB();
    }

    public void set(double value) {
        setL(value);
        setR(value);
    }

    public void setL(double value) {
        _leftSparkMax.set(value);
    }

    public void setR(double value) {
        _rightSparkMax.set(value);
    }

    public void setHighClimb(double value){
        this._highClimbSparkMax.set(value);
    }

    private double getLeftPosition() {
        return this._leftSparkMax.getPosition();
    }

    private double getRightPosition() {
        return this._rightSparkMax.getPosition();
    }

    private double getHighClimbPosition(){
        return this._highClimbSparkMax.getPosition();
    }

    private void initSDB() {
        // SmartDashboard.setDefaultNumber("mid soft limit forward", Constants.Climber.climbExtentionHeight);
        // SmartDashboard.setDefaultNumber("high soft limit forward", Constants.Climber.climbExtentionHeight);
        // SmartDashboard.setDefaultNumber("soft limit reverse", Constants.Climber.startingPosition);

        SmartDashboard.putData("enable left soft limit", new InstantCommand(() -> enableSoftLimit(true)));
        SmartDashboard.putData("disable left soft limit", new InstantCommand(() -> enableSoftLimit(false)));
        // SmartDashboard.putData("update soft limit position",
        //         new InstantCommand(() -> updateSoftLimitPositionFromSDB()));
    }

    private void enableSoftLimit(boolean enabled) {
        this._leftSparkMax.enableSoftLimit(SoftLimitDirection.kForward, enabled);
        this._leftSparkMax.enableSoftLimit(SoftLimitDirection.kReverse, enabled);
        this._rightSparkMax.enableSoftLimit(SoftLimitDirection.kForward, enabled);
        this._rightSparkMax.enableSoftLimit(SoftLimitDirection.kReverse, enabled);
        this._highClimbSparkMax.enableSoftLimit(SoftLimitDirection.kForward, enabled);
        this._highClimbSparkMax.enableSoftLimit(SoftLimitDirection.kReverse, enabled);
    }

    @SuppressWarnings("unused")
    private void updateSoftLimitPositionFromSDB() {
        float reverseLimit = (float) SmartDashboard.getNumber("soft limit reverse",
                Constants.Climber.startingPosition);
        float midForwardLimit = (float) SmartDashboard.getNumber("mid soft limit forward",
                Constants.Climber.midClimbExtentionHeight);
        float highForwardLimit = (float) SmartDashboard.getNumber("high soft limit forward",
                Constants.Climber.midClimbExtentionHeight);
        updateSoftLimitPosition(reverseLimit, midForwardLimit, highForwardLimit);
    }

    private void updateSoftLimitPosition(float reverseLimit, float midForwardLimit, float highForwardLimit) {
        this._rightSparkMax.setSoftLimit(SoftLimitDirection.kReverse, reverseLimit);
        this._rightSparkMax.setSoftLimit(SoftLimitDirection.kForward, midForwardLimit);
        this._leftSparkMax.setSoftLimit(SoftLimitDirection.kReverse, reverseLimit);
        this._leftSparkMax.setSoftLimit(SoftLimitDirection.kForward, midForwardLimit);
        this._highClimbSparkMax.setSoftLimit(SoftLimitDirection.kReverse, reverseLimit);
        this._highClimbSparkMax.setSoftLimit(SoftLimitDirection.kForward, highForwardLimit);
    }

    private void updateSDB() {
        SmartDashboard.putNumber("Climber Left Position", getLeftPosition());
        SmartDashboard.putNumber("Climber Right Position", getRightPosition());
        SmartDashboard.putNumber("high climber position", getHighClimbPosition());
    }

    public void disableInit() {
        set(0);
        setHighClimb(0);
    }

    @Override
    public void periodic() {
        updateSDB();
    }
}
