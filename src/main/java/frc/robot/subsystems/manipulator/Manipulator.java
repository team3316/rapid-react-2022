// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.manipulator;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.motors.ControlMode;
import frc.robot.motors.DBugTalon;
import frc.robot.motors.EncoderModel;
import frc.robot.motors.IDBugMotorController;
import frc.robot.motors.TalonModel;
import frc.robot.motors.units.UnitConversions;
import frc.robot.motors.units.VelocityUnit;

public class Manipulator extends SubsystemBase {

  public enum ManipulatorState{
    COLLECT(Constants.Manipulator.collectRPM),
    OFF(0);

    public final double rpm; 
    private ManipulatorState(double rpm){
      this.rpm = rpm;
    }
  }
  /** Creates a new Intake. */
  private DBugTalon _leaderMotor;
  private DBugTalon _followerMotor;
  private DigitalInput _leftSwitch;
  private DigitalInput _rightSwitch;
  public Manipulator() {
    this._leaderMotor = new DBugTalon(Constants.Manipulator.leaderId, TalonModel.TalonFX, new UnitConversions(1,3*2.54/100,EncoderModel.FalconInternal), FeedbackDevice.IntegratedSensor);
    this._followerMotor = new DBugTalon(Constants.Manipulator.followerId,TalonModel.TalonFX, new UnitConversions(1,3*2.54/100,EncoderModel.FalconInternal), FeedbackDevice.IntegratedSensor);
    this._leftSwitch = new DigitalInput(Constants.Manipulator.leftSwitchId);
    this._rightSwitch = new DigitalInput(Constants.Manipulator.rightSwitchId);
    this._followerMotor.follow((IDBugMotorController)this._leaderMotor);
    this._followerMotor.setInverted(true);
    this._leaderMotor.setupPIDF(Constants.Manipulator.gains);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("realVel", _leaderMotor.getVelocity(VelocityUnit.RPM));
    SmartDashboard.putNumber("wantedVel", SmartDashboard.getNumber("wantedVel", 0));
    SmartDashboard.putNumber("kP", SmartDashboard.getNumber("kP", 0));
    SmartDashboard.putNumber("kI", SmartDashboard.getNumber("kP", 0));
    SmartDashboard.putNumber("kD", SmartDashboard.getNumber("kD", 0));
    this._leaderMotor.config_kP(0, SmartDashboard.getNumber("kP", 0));
    this._leaderMotor.config_kI(0, SmartDashboard.getNumber("kP", 0));
    this._leaderMotor.config_kD(0, SmartDashboard.getNumber("kD", 0));
    // This method will be called once per scheduler run
  }

  public void setState(ManipulatorState state){
    this._leaderMotor.set(ControlMode.Velocity, state.rpm, VelocityUnit.RPM);
  }

  public ManipulatorCargoState getCargoState(){
    return new ManipulatorCargoState(this._leftSwitch.get(),this._rightSwitch.get());
  }

  
  public void set(double rpm){
    this._leaderMotor.set(ControlMode.Velocity,rpm,VelocityUnit.RPM);
  }

}
