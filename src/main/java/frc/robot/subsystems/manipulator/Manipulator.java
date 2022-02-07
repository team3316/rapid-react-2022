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

public class Manipulator extends SubsystemBase {

    public enum ManipulatorState {
        COLLECT(Constants.Manipulator.collectRPM),
        SHOOT(Constants.Manipulator.shootRPM),
        OFF(0);

        public final double rpm;

        private ManipulatorState(double rpm) {
            this.rpm = rpm;
        }
    }

    private TalonFX _leaderMotor;
    private TalonFX _followerMotor;
    private DigitalInput _leftSwitch;
    private DigitalInput _rightSwitch;
    private TalonFXConfiguration _leaderConfig;

    public Manipulator() {
        this._leaderMotor = new TalonFX(Constants.Manipulator.leaderId);
        this._followerMotor = new TalonFX(Constants.Manipulator.followerId);

        this._leftSwitch = new DigitalInput(Constants.Manipulator.leftChannel);
        this._rightSwitch = new DigitalInput(Constants.Manipulator.rightChannel);

        _leaderConfig = new TalonFXConfiguration();

        this._followerMotor.follow(this._leaderMotor);
        this._followerMotor.setInverted(InvertType.OpposeMaster);

        _leaderMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 20);

        _leaderConfig.slot0.kF = Constants.Manipulator.kF;
        _leaderConfig.slot0.kP = Constants.Manipulator.kP;
        _leaderConfig.slot0.closedLoopPeakOutput = Constants.Manipulator.kPeakOutput;
        _leaderConfig.closedloopRamp = Constants.Manipulator.maxAccelerationSeconds;

        _leaderMotor.configAllSettings(_leaderConfig);

        initSDB();

    }

    @Override
    public void periodic() {
        updateSDB();
    }

    public void setState(ManipulatorState state) {
        setTargetRPM(state.rpm);
    }

    public ManipulatorCargoState getCargoState() {
        return new ManipulatorCargoState(!this._leftSwitch.get(), !this._rightSwitch.get());
    }

    private void setTargetRPM(double rpm) {
        _leaderMotor.set(TalonFXControlMode.Velocity, (rpm / Constants.Manipulator.kVelocityConversionFactor));
    }

    private double getTargetRPM() {
        return _leaderMotor.getClosedLoopTarget() * Constants.Manipulator.kVelocityConversionFactor;
    }

    private double getRPM() {
        return _leaderMotor.getSelectedSensorVelocity() * Constants.Manipulator.kVelocityConversionFactor;
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
        SmartDashboard.putNumber("Manipulator Velocity", getRPM());

        SmartDashboard.putNumber("Manipulator Target", getTargetRPM());

        SmartDashboard.putBoolean("left switch", !this._leftSwitch.get());
        SmartDashboard.putBoolean("right switch", !this._rightSwitch.get());
        SmartDashboard.putBoolean("both switches", getCargoState().hasBoth());
    }

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

        updateSDB();

    }
}
