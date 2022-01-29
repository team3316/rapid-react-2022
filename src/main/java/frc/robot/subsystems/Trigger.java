// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.motors.BetterServo;

public class Trigger extends SubsystemBase {
  
  private BetterServo _servoLeft, _servoRight;

  public Trigger() {
    this._servoLeft = new BetterServo(Constants.Trigger.CHANNEL_LEFT);
    this._servoRight = new BetterServo(Constants.Trigger.CHANNEL_RIGHT);
  }

  public enum TriggerState {
    IN(Constants.Trigger.TriggerState.IN_POS), OUT(Constants.Trigger.TriggerState.OUT_POS);

    public final double angle;

    TriggerState(double angle){
      this.angle = angle;
    }
  }

  public enum Side{
    LEFT, RIGHT;
  }

  public void setState(TriggerState state, Side side){
    if(side == Side.LEFT){
      this._servoLeft.setAngle(state.angle);
    }
    else{
      this._servoRight.setAngle(state.angle);
    }
    
    
  }

  public TriggerState getState(Side side){

    if(side == Side.LEFT){
      if(this._servoLeft.getAngle() == TriggerState.IN.angle){
        return TriggerState.IN;
      }
      return TriggerState.OUT;
    }

    if(this._servoRight.getAngle() == TriggerState.IN.angle){
      return TriggerState.IN;
    }
    return TriggerState.OUT;
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
