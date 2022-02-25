// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.motors.DBugSparkMax;

public class Climber extends SubsystemBase {

    private DBugSparkMax _leftSparkMax, _rightSparkMax;

    private static DBugSparkMax createSparkMax(int id) {
        DBugSparkMax sparkMax = new DBugSparkMax(id);
        sparkMax.restoreFactoryDefaults();
        sparkMax.setConversionFactors(Constants.Climber.conversionFactor, Constants.Climber.conversionFactor / 60);
        sparkMax.setSmartCurrentLimit(40);
        sparkMax.enableVoltageCompensation(12);
        sparkMax.setIdleMode(IdleMode.kBrake);
        sparkMax.enableSoftLimit(SoftLimitDirection.kForward, true);
        sparkMax.enableSoftLimit(SoftLimitDirection.kReverse, true);
        sparkMax.setSoftLimit(SoftLimitDirection.kForward, (float) Constants.Climber.climbExtentionHeight);
        sparkMax.setSoftLimit(SoftLimitDirection.kReverse, (float) Constants.Climber.startingPosition);
        sparkMax.setPosition(Constants.Climber.startingPosition);
        return sparkMax;
    }

    public Climber() {
        this._leftSparkMax = createSparkMax(Constants.Climber.leftID);
        this._rightSparkMax = createSparkMax(Constants.Climber.rightID);

        this._rightSparkMax.setInverted(false);

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

    private double getLeftPosition() {
        return this._leftSparkMax.getPosition();
    }

    private double getRightPosition() {
        return this._rightSparkMax.getPosition();
    }

    private void initSDB() {
        SmartDashboard.setDefaultNumber("soft limit forward", Constants.Climber.climbExtentionHeight);
        SmartDashboard.setDefaultNumber("soft limit reverse", Constants.Climber.startingPosition);

        SmartDashboard.putData("enable left soft limit", new InstantCommand(() -> enableSoftLimit(true)));
        SmartDashboard.putData("disable left soft limit", new InstantCommand(() -> enableSoftLimit(false)));
        SmartDashboard.putData("update soft limit position", new InstantCommand(() -> updateSoftLimitPosition()));
    }

    private void enableSoftLimit(boolean enabled) {
        this._leftSparkMax.enableSoftLimit(SoftLimitDirection.kForward, enabled);
        this._leftSparkMax.enableSoftLimit(SoftLimitDirection.kReverse, enabled);
        this._rightSparkMax.enableSoftLimit(SoftLimitDirection.kForward, enabled);
        this._rightSparkMax.enableSoftLimit(SoftLimitDirection.kReverse, enabled);
    }

    private void updateSoftLimitPosition() {
        float reverseLimit = (float) SmartDashboard.getNumber("soft limit reverse",
                Constants.Climber.startingPosition);
        float forwardLimit = (float) SmartDashboard.getNumber("soft limit forward",
                Constants.Climber.climbExtentionHeight);
        this._rightSparkMax.setSoftLimit(SoftLimitDirection.kReverse, reverseLimit);
        this._rightSparkMax.setSoftLimit(SoftLimitDirection.kForward, forwardLimit);
        this._leftSparkMax.setSoftLimit(SoftLimitDirection.kReverse, reverseLimit);
        this._leftSparkMax.setSoftLimit(SoftLimitDirection.kForward, forwardLimit);
    }

    private void updateSDB() {
        SmartDashboard.putNumber("Climber Left Position", getLeftPosition());
        SmartDashboard.putNumber("Climber Right Position", getRightPosition());
    }

    public void disableInit() {
        set(0);
    }

    @Override
    public void periodic() {
        updateSDB();
    }
}
