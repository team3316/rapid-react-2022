// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  private TalonFX _leaderMotor;
  private TalonFX _followerMotor;
  private DigitalInput _leftSwitch;
  private DigitalInput _rightSwitch;
  public Intake() {
    this._leaderMotor = new TalonFX(Constants.Intake.leaderId);
    this._followerMotor = new TalonFX(Constants.Intake.followerId);
    this._leftSwitch = new DigitalInput(Constants.Intake.leftSwitchId);
    this._rightSwitch = new DigitalInput(Constants.Intake.rightSwitchId);
    this._followerMotor.follow(this._leaderMotor);
    this._followerMotor.setInverted(TalonFXInvertType.OpposeMaster);
    _leaderMotor.config_kP(0, Constants.Intake.kP);
    _leaderMotor.config_kI(0, Constants.Intake.kI);
    _leaderMotor.config_kD(0, Constants.Intake.kD);
    _leaderMotor.setNeutralMode(NeutralMode.Brake);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setRPM(double RPM){
    _leaderMotor.set(ControlMode.Velocity, RPM/600*2048*(24/16));
  }

  public void setZero(){
    _leaderMotor.setNeutralMode(NeutralMode.Coast);
    _leaderMotor.set(ControlMode.PercentOutput, 0);
  }

  public IntakeBallState getCargoState(){
    return new IntakeBallState(_leftSwitch.get(),_rightSwitch.get());
  }

}
