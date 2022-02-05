// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.manipulator;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Manipulator extends SubsystemBase {

  public enum ManipulatorState{
    COLLECT(Constants.Manipulator.collectRPM),
    SHOOT(Constants.Manipulator.shootRPM),
    OFF(0);

    public final double rpm; 
    private ManipulatorState(double rpm){
      this.rpm = rpm;
    }
  }
  /** Creates a new Intake. */
  private TalonFX _leaderMotor;
  private TalonFX _followerMotor;
  private DigitalInput _leftSwitch;
  private DigitalInput _rightSwitch;
  private TalonFXConfiguration configs;
  private TalonFXConfiguration _leaderConfig;
  private TalonFXConfiguration _followerConfig;

  public Manipulator() {
    this._leaderMotor = new TalonFX(Constants.Manipulator.leaderId);
    this._followerMotor = new TalonFX(Constants.Manipulator.followerId);

    configs = new TalonFXConfiguration();
    _leaderConfig = new TalonFXConfiguration();
    _followerConfig = new TalonFXConfiguration();

    this._followerMotor.follow(this._leaderMotor);
    this._followerMotor.setInverted(InvertType.OpposeMaster);

    configs.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
    
    _leaderMotor.configAllSettings(configs);
    _leaderMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 20);

		_leaderConfig.primaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice();

    _leaderConfig.slot0.kF = SmartDashboard.getNumber("Manipulator kF", Constants.Manipulator.kF);
    _leaderConfig.slot0.kP = SmartDashboard.getNumber("Manipulator kP", Constants.Manipulator.kP);
		_leaderConfig.slot0.closedLoopPeakOutput = SmartDashboard.getNumber("Manipulator Peak Output", Constants.Manipulator.kPeakOutput);
    _leaderConfig.closedloopRamp = SmartDashboard.getNumber("Manipulator Max Acceleration Seconds", Constants.Manipulator.maxAccelerationSeconds);

		_leaderMotor.configAllSettings(_leaderConfig);
		_followerMotor.configAllSettings(_followerConfig);

    initSDB();

  }

  @Override
  public void periodic() { 
    // This method will be called once per scheduler run
  }

  public void setState(ManipulatorState state) {
    _leaderMotor.set(TalonFXControlMode.Velocity, (state.rpm / Constants.Manipulator.kVelocityConversionFactor));
  }

  public ManipulatorCargoState getCargoState(){
    return new ManipulatorCargoState(this._leftSwitch.get(),this._rightSwitch.get());
  }

  public void setPID(){
    _leaderConfig.slot0.kF = SmartDashboard.getNumber("Manipulator kF", Constants.Manipulator.kF);
    _leaderConfig.slot0.kP = SmartDashboard.getNumber("Manipulator kP", Constants.Manipulator.kP);
  }

  public void setTarget(){
    _leaderMotor.set(TalonFXControlMode.Velocity, (SmartDashboard.getNumber("Manipulator Target", _leaderMotor.getClosedLoopTarget() * Constants.Manipulator.kVelocityConversionFactor)));
  }

  private void updateSDB() {
    SmartDashboard.putNumber("Manipulator Velocity", _leaderMotor.getSelectedSensorVelocity() * Constants.Manipulator.kVelocityConversionFactor);
    SmartDashboard.putData("Set kF", new InstantCommand(() -> setPID()));
    SmartDashboard.putData("Set target", new InstantCommand(() -> setTarget()));
  }

  private void initSDB() {
    SmartDashboard.setDefaultNumber("Manipulator Peak Output", Constants.Manipulator.kPeakOutput);
    SmartDashboard.setDefaultNumber("Manipulator Max Acceleration Seconds", Constants.Manipulator.maxAccelerationSeconds);
    SmartDashboard.setDefaultNumber("Manipulator kF", Constants.Manipulator.kF);
    SmartDashboard.setDefaultNumber("Manipulator kP", Constants.Manipulator.kP);
    SmartDashboard.putNumber("Manipulator Target", _leaderMotor.getClosedLoopTarget() * Constants.Manipulator.kVelocityConversionFactor);


    SmartDashboard.putData("Update Manipulator from SDB", new InstantCommand(() -> updateSDB()));
  }
}
