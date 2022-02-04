// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
    private CANSparkMax _leaderSM;
    private CANSparkMax _followerSM;
    private SparkMaxLimitSwitch _forwardLimit;
    private SparkMaxLimitSwitch _reverseLimit;
    private ArmFeedforward _armFeedforward;
  /** Creates a new Arm. */
  public Arm() {
    _leaderSM = new CANSparkMax(16,MotorType.kBrushless);
    _followerSM = new CANSparkMax(17,MotorType.kBrushless);
    _followerSM.follow((CANSparkMax)_leaderSM, true);
    _followerSM.setIdleMode(IdleMode.kBrake);
    _leaderSM.setIdleMode(IdleMode.kBrake);
    _forwardLimit = _leaderSM.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
    _reverseLimit = _leaderSM.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
    _forwardLimit.enableLimitSwitch(true);
    _reverseLimit.enableLimitSwitch(true);
    // _leaderSM.getEncoder().setPositionConversionFactor((1.0 / 33.6) * 360 * -1);
    _leaderSM.getEncoder().setPositionConversionFactor(360/33.6);
    _leaderSM.getEncoder().setVelocityConversionFactor(360/33.6);
    this._leaderSM.setInverted(false);
    this._armFeedforward = new ArmFeedforward(0, SmartDashboard.getNumber("precent", 0), 0);
  }

  public void setPrecent(double precent){
    this._leaderSM.set(Math.min(Math.max(precent, -0.1), 0.1));
    SmartDashboard.putNumber("precent given", Math.min(Math.max(precent, -0.1), 0.1));
  }

  public void setFFPrecent(){
    double precent = this._armFeedforward.calculate(Math.toRadians(SmartDashboard.getNumber("real angle", 0)), 0);
    setPrecent(-precent);
  }

  public void setKCos(){
    this._armFeedforward = new ArmFeedforward(0, SmartDashboard.getNumber("precent", 0), 0);
  }
  public double getPrecent(){
    return this._leaderSM.get();
  }

  public double getCurrentVoltage(){
    return this._leaderSM.getEncoder().getVelocity();
  }

  public double getAngle(){
    return this._leaderSM.getEncoder().getPosition();
  }

  @Override
  public void periodic() {
    if(_forwardLimit.isPressed()){
      this._leaderSM.getEncoder().setPosition(37);
    }
    else if(_reverseLimit.isPressed()){
      this._leaderSM.getEncoder().setPosition(-116);
    }
    SmartDashboard.putNumber("encoder position", this._leaderSM.getEncoder().getPosition());
    SmartDashboard.putBoolean("fwrd", _forwardLimit.isPressed());
    SmartDashboard.putBoolean("bwrd", _reverseLimit.isPressed());
  }
}
