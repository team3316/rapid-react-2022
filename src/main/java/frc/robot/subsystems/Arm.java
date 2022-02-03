// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.motors.ControlMode;
import frc.robot.motors.DBugSparkMax;
import frc.robot.motors.units.UnitConversions;

public class Arm extends SubsystemBase {
  private DBugSparkMax _leaderSM;
    private DBugSparkMax _followerSM;
    private SparkMaxLimitSwitch _forwardLimit;
    private SparkMaxLimitSwitch _reverseLimit;
  /** Creates a new Arm. */
  public Arm() {
    _leaderSM = new DBugSparkMax(16, new UnitConversions(1/1/33.6));
    _followerSM = new DBugSparkMax(17, new UnitConversions(1/1/33.6));
    _followerSM.follow((CANSparkMax)_leaderSM, true);
    _followerSM.setIdleMode(IdleMode.kBrake);
    _leaderSM.setIdleMode(IdleMode.kBrake);
    _forwardLimit = _leaderSM.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
    _reverseLimit = _leaderSM.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
    _forwardLimit.enableLimitSwitch(true);
    _reverseLimit.enableLimitSwitch(true);
    // _leaderSM.getEncoder().setPositionConversionFactor((1.0 / 33.6) * 360 * -1);
    _leaderSM.getEncoder().setPositionConversionFactor(1.0);
  }

  public void setPrecent(double precent){
    this._leaderSM.set(ControlMode.PercentOutput, Math.max(precent, -0.1));
  }

  public double getPrecent(){
    return this._leaderSM.get();
  }

  public double getCurrentVoltage(){
    return this._leaderSM.getVoltageCompensationNominalVoltage();
  }

  public double getAngle(){
    return this._leaderSM.getEncoder().getPosition();
  }

  @Override
  public void periodic() {
    if(_forwardLimit.isPressed()){
      this._leaderSM.setPosition(-37.0);
    }
    else if(_reverseLimit.isPressed()){
      this._leaderSM.setPosition(126.0);
    }
    SmartDashboard.putNumber("encoder position", this._leaderSM.getEncoder().getPosition());
  }
}
