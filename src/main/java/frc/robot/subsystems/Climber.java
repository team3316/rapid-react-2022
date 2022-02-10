// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.motors.ControlMode;
import frc.robot.motors.DBugSparkMax;
import frc.robot.motors.PIDFGains;
import frc.robot.motors.units.UnitConversions;

public class Climber extends SubsystemBase {
  
  private DBugSparkMax _leftSparkMax, _rightSparkMax;

  public Climber() {
    this._leftSparkMax = new DBugSparkMax(Constants.Climber.deviceNumberLeft, new UnitConversions(Constants.Climber.gearRatio));
    this._rightSparkMax = new DBugSparkMax(Constants.Climber.deviceNumberRight, new UnitConversions(Constants.Climber.gearRatio));

    this._leftSparkMax.restoreFactoryDefaults();
    this._rightSparkMax.restoreFactoryDefaults();

    this._rightSparkMax.follow(this._leftSparkMax, Constants.Climber.invert);

    this._leftSparkMax.setupPIDF(new PIDFGains(Constants.Climber.kP, 0, 0, Constants.Climber.kF, 0, 0));
  }

  public void setPrecent(double precent) {
    this._leftSparkMax.set(ControlMode.PercentOutput, precent);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
