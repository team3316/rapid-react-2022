// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.manipulator;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.DoubleLatchedBoolean;
import frc.robot.utils.Within;

public class Manipulator extends SubsystemBase {

    public enum ManipulatorState {
        COLLECT(Constants.Manipulator.collectRPM),
        SHOOT(Constants.Manipulator.shootRPM),
        SHOOTLOW(Constants.Manipulator.shootLowRPM),
        OFF(0);

        public final double rpm;

        private ManipulatorState(double rpm) {
            this.rpm = rpm;
        }
    }

    private TalonFX _leaderMotor;
    private TalonFX _followerMotor;
    private DigitalInput _leftSwitch;
    private DoubleLatchedBoolean _leftSwitchBoolean;
    private DigitalInput _rightSwitch;
    private DoubleLatchedBoolean _rightSwitchBoolean;
    private TalonFXConfiguration _leaderConfig;

    public Manipulator() {
        this._leaderMotor = new TalonFX(Constants.Manipulator.leaderId);
        this._followerMotor = new TalonFX(Constants.Manipulator.followerId);

        this._leftSwitch = new DigitalInput(Constants.Manipulator.leftChannel);
        _leftSwitchBoolean = new DoubleLatchedBoolean();
        this._rightSwitch = new DigitalInput(Constants.Manipulator.rightChannel);
        _rightSwitchBoolean = new DoubleLatchedBoolean();

        _leaderConfig = new TalonFXConfiguration();

        _followerMotor.configAllSettings(new TalonFXConfiguration());

        this._followerMotor.follow(this._leaderMotor);
        this._followerMotor.setInverted(InvertType.OpposeMaster);

        _leaderMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 20);

        _leaderConfig.slot0.kF = Constants.Manipulator.kF;
        _leaderConfig.slot0.kP = Constants.Manipulator.kP;
        _leaderConfig.slot0.closedLoopPeakOutput = Constants.Manipulator.kPeakOutput;
        _leaderConfig.closedloopRamp = Constants.Manipulator.maxAccelerationSeconds;
        _leaderConfig.voltageCompSaturation = 10.0; // volts

        _leaderMotor.configAllSettings(_leaderConfig);

        _leaderMotor.enableVoltageCompensation(true);
        // initSDB();

    }

    @Override
    public void periodic() {
        updateSDB();
    }

    public void setState(ManipulatorState state) {
        if (state == ManipulatorState.OFF)
            _leaderMotor.set(TalonFXControlMode.PercentOutput, 0);
        else
            setTargetRPM(state.rpm);
    }

    public ManipulatorCargoState getCargoState() {
        return new ManipulatorCargoState(!this._leftSwitch.get(), !this._rightSwitch.get());
    }

    private void setTargetRPM(double rpm) {
        _leaderMotor.set(TalonFXControlMode.Velocity, (rpm /
                Constants.Manipulator.kVelocityConversionFactor));

        SmartDashboard.putNumber("Manipulator Target", getTargetRPM());
    }

    private double getTargetRPM() {
        return _leaderMotor.getClosedLoopTarget() *
                Constants.Manipulator.kVelocityConversionFactor;
    }

    private double getRPM() {
        return _leaderMotor.getSelectedSensorVelocity() *
                Constants.Manipulator.kVelocityConversionFactor;
    }

    private void updateConfig() {
        _leaderConfig.slot0.kF = SmartDashboard.getNumber(
                "Manipulator kF",
                Constants.Manipulator.kF);
        _leaderConfig.slot0.kP = SmartDashboard.getNumber(
                "Manipulator kP",
                Constants.Manipulator.kP);

        _leaderConfig.slot0.closedLoopPeakOutput = SmartDashboard.getNumber(
                "Manipulator Peak Output",
                Constants.Manipulator.kPeakOutput);

        _leaderConfig.closedloopRamp = SmartDashboard.getNumber(
                "Manipulator Max Acceleration Seconds",
                Constants.Manipulator.maxAccelerationSeconds);

        _leaderMotor.configAllSettings(_leaderConfig);
    }

    private void setTarget() {
        setTargetRPM(SmartDashboard.getNumber("Manipulator Target", getTargetRPM()));
    }

    private void updateSDB() {
        if (!Within.range(_leaderMotor.getStatorCurrent(), 0, 0.0001))
        SmartDashboard.putNumber("Manipulator Velocity", getRPM());

        ManipulatorCargoState state = getCargoState();

        if (_leftSwitchBoolean.update(state.leftCargo))
            SmartDashboard.putBoolean("Manipulator Left Switch", state.leftCargo);

        if (_rightSwitchBoolean.update(state.rightCargo))
            SmartDashboard.putBoolean("Manipulator Right Switch", state.rightCargo);
    }

    @SuppressWarnings({ "unused" })
    private void initSDB() {
        SmartDashboard.setDefaultNumber(
                "Manipulator Peak Output",
                Constants.Manipulator.kPeakOutput);

        SmartDashboard.setDefaultNumber(
                "Manipulator Max Acceleration Seconds",
                Constants.Manipulator.maxAccelerationSeconds);

        SmartDashboard.setDefaultNumber(
                "Manipulator kF",
                Constants.Manipulator.kF);

        SmartDashboard.setDefaultNumber(
                "Manipulator kP",
                Constants.Manipulator.kP);

        SmartDashboard.putData(
                "Set target",
                new InstantCommand(() -> setTarget()));

        SmartDashboard.putData(
                "Update Manipulator from SDB",
                new InstantCommand(() -> updateConfig()));

    }
}
