// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.motors.ControlMode;
import frc.robot.motors.DBugSparkMax;
import frc.robot.motors.PIDFGains;
import frc.robot.motors.units.PositionUnit;
import frc.robot.motors.units.UnitConversions;

public class Climber extends SubsystemBase {

    private DBugSparkMax _leftSparkMax, _rightSparkMax;

    private double _lastPosition;

    public Climber() {
        this._leftSparkMax = new DBugSparkMax(Constants.Climber.leftID,
                new UnitConversions(Constants.Climber.gearRatio));
        this._rightSparkMax = new DBugSparkMax(Constants.Climber.rightID,
                new UnitConversions(Constants.Climber.gearRatio));

        this._leftSparkMax.restoreFactoryDefaults();
        this._rightSparkMax.restoreFactoryDefaults();

        this._leftSparkMax.setIdleMode(IdleMode.kBrake);
        this._rightSparkMax.setIdleMode(IdleMode.kBrake);

        this._rightSparkMax.follow(this._leftSparkMax, true);

        this._leftSparkMax.setupPIDF(new PIDFGains(Constants.Climber.kP_slot0, 0, 0, Constants.Climber.kF_slot0, 0, 0));
        this._leftSparkMax.getPIDController().setP(Constants.Climber.kP_slot1, Constants.Climber.PIDslot);
        this._leftSparkMax.getPIDController().setI(0, Constants.Climber.PIDslot);
        this._leftSparkMax.getPIDController().setD(0, Constants.Climber.PIDslot);
        this._leftSparkMax.getPIDController().setFF(Constants.Climber.kF_slot1, Constants.Climber.PIDslot);
        this._leftSparkMax.getPIDController().setOutputRange(-1.0, 1.0, Constants.Climber.PIDslot);

        this._leftSparkMax.getEncoder().setPositionConversionFactor(Constants.Climber.conversionFactor);

        initSDB();
    }

    public enum ClimberState {
        // TODO: add real values
        UP(0.0), DOWN(0.0);

        public final double position;

        ClimberState(double position) {
            this.position = position;
        }
    }

    public void setPosition(ClimberState state) {
        this._lastPosition = state.position;
        this._leftSparkMax.set(ControlMode.Position, state.position);
    }

    public boolean isLastPositionDown() {
        return (this._lastPosition == ClimberState.DOWN.position);
    }

    private double getPosition() {
        return this._leftSparkMax.getPosition(PositionUnit.Rotations);
    }

    private void initSDB() {
        SmartDashboard.setDefaultNumber("kP loaded behavior", Constants.Climber.kP_slot0);
        SmartDashboard.setDefaultNumber("kF loaded behavior", Constants.Climber.kF_slot0);

        SmartDashboard.putData("update pidf", new InstantCommand(() -> updatePIDF()));
    }

    private void updateSDB() {
        SmartDashboard.putNumber("Climber Position", getPosition());
    }

    private void updatePIDF() {
        this._leftSparkMax.setupPIDF(new PIDFGains(SmartDashboard.getNumber("kP", Constants.Climber.kP_slot0), 0, 0,
                SmartDashboard.getNumber("kF", Constants.Climber.kF_slot0), 0, 0));
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        updateSDB();
    }
}
