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
    IN(Constants.Trigger.TriggerState.IN_ANGLE_LEFT, Constants.Trigger.TriggerState.IN_ANGLE_RIGHT), OUT(Constants.Trigger.TriggerState.OUT_ANGLE_LEFT, Constants.Trigger.TriggerState.OUT_ANGLE_RIGHT);

    public final double leftAngle;
    public final double rightAngle;

    TriggerState(double leftAngle, double rightAngle){
      this.leftAngle = leftAngle;
      this.rightAngle = rightAngle;
    }
  }

  public enum Side{
    LEFT, RIGHT;
  }

  public void setState(TriggerState state, Side side){
    switch (side) {
      case LEFT:
        this._servoLeft.setAngle(state.leftAngle);
        break;
      case RIGHT:
        this._servoRight.setAngle(state.rightAngle);
        break;
      default:
        break;
    }
  }

  public TriggerState getState(Side side){

    switch (side) {
      case LEFT:
        if(Math.abs(this._servoLeft.getAngle() - TriggerState.OUT.leftAngle) <= 0.00001){
          return TriggerState.OUT;
        }
        return TriggerState.IN;
      case RIGHT:
        if(Math.abs(this._servoRight.getAngle() - TriggerState.OUT.rightAngle) <= 0.00001){
          return TriggerState.OUT;
        }
        return TriggerState.IN;
      default:
        return null;
      }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
