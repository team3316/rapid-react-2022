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

public class Manipulator extends SubsystemBase {

  public enum ManipulatorState{
    SHOOT(Constants.Manipulator.shootRPM),
    COLLECT(Constants.Manipulator.collectRPM),
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
  public Manipulator() {
    this._leaderMotor = new TalonFX(Constants.Manipulator.leaderId);
    this._followerMotor = new TalonFX(Constants.Manipulator.followerId);
    this._leftSwitch = new DigitalInput(Constants.Manipulator.leftSwitchId);
    this._rightSwitch = new DigitalInput(Constants.Manipulator.rightSwitchId);
    this._followerMotor.follow(this._leaderMotor);
    this._followerMotor.setInverted(TalonFXInvertType.OpposeMaster);
    _leaderMotor.config_kP(0, Constants.Manipulator.kP);
    _leaderMotor.config_kI(0, Constants.Manipulator.kI);
    _leaderMotor.config_kD(0, Constants.Manipulator.kD);
    // _leaderMotor.setNeutralMode(NeutralMode.Brake);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setState(ManipulatorState state){
    _leaderMotor.set(ControlMode.Velocity, state.rpm/600*2048*(24/16));
  }

  public void setZero(){
    _leaderMotor.setNeutralMode(NeutralMode.Coast);
    _leaderMotor.set(ControlMode.PercentOutput, 0);
  }

  public ManipulatorCargoState getCargoState(){
    return new ManipulatorCargoState(_leftSwitch.get(),_rightSwitch.get());
  }

}
