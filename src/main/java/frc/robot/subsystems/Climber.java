// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.SparkMaxPIDController.AccelStrategy;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.motors.DBugSparkMax;
import frc.robot.motors.PIDFGains;
import frc.robot.motors.units.PositionUnit;
import frc.robot.motors.units.UnitConversions;

public class Climber extends SubsystemBase {

    private DBugSparkMax _leftSparkMax, _rightSparkMax;

    private SparkMaxPIDController _PidController;

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

        this._PidController = _leftSparkMax.getPIDController();
        this._leftSparkMax.setupPIDF(new PIDFGains(Constants.Climber.Down.kP, 0, 0, Constants.Climber.Down.kF, 0, 0));
        this._PidController.setP(Constants.Climber.Up.kP, Constants.Climber.Up.PIDslot);
        this._PidController.setI(0, Constants.Climber.Up.PIDslot);
        this._PidController.setD(0, Constants.Climber.Up.PIDslot);
        this._PidController.setFF(Constants.Climber.Up.kF, Constants.Climber.Up.PIDslot);
        this._PidController.setOutputRange(-1.0, 1.0, Constants.Climber.Up.PIDslot);

        this._PidController.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, Constants.Climber.Up.PIDslot);
        this._PidController.setSmartMotionAllowedClosedLoopError(Constants.Climber.Up.allowedError, Constants.Climber.Up.PIDslot);
        this._PidController.setSmartMotionMaxAccel(Constants.Climber.Up.maxAccMetersPerSecondSqrd, Constants.Climber.Up.PIDslot);
        this._PidController.setSmartMotionMaxVelocity(Constants.Climber.Up.maxVelMetersPerSecond, Constants.Climber.Up.PIDslot);
        this._PidController.setSmartMotionMinOutputVelocity(Constants.Climber.Up.minVelMetersPerSecond, Constants.Climber.Up.PIDslot);

        this._PidController.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, Constants.Climber.Down.PIDslot);
        this._PidController.setSmartMotionAllowedClosedLoopError(Constants.Climber.Down.allowedError, Constants.Climber.Down.PIDslot);
        this._PidController.setSmartMotionMaxAccel(Constants.Climber.Down.maxAccMetersPerSecondSqrd, Constants.Climber.Down.PIDslot);
        this._PidController.setSmartMotionMaxVelocity(Constants.Climber.Down.maxVelMetersPerSecond, Constants.Climber.Down.PIDslot);
        this._PidController.setSmartMotionMinOutputVelocity(Constants.Climber.Down.minVelMetersPerSecond, Constants.Climber.Down.PIDslot);

        this._leftSparkMax.getEncoder().setPositionConversionFactor(Constants.Climber.conversionFactor);

        initSDB();
    }

    public enum ClimberState {
        // TODO: add real values
        UP(0.0, Constants.Climber.Up.PIDslot), DOWN(0.0, Constants.Climber.Down.PIDslot);

        public final double position;
        public final int slot;

        ClimberState(double position, int slot) {
            this.position = position;
            this.slot = slot;
        }
    }

    public void setPosition(ClimberState state) {
        this._lastPosition = state.position;
        this._PidController.setReference(state.position, ControlType.kSmartMotion, state.slot);
    }

    public boolean isLastPositionDown() {
        return (this._lastPosition == ClimberState.DOWN.position);
    }

    private double getPosition() {
        return this._leftSparkMax.getPosition(PositionUnit.Rotations);
    }

    private void initSDB() {
        SmartDashboard.setDefaultNumber("kP loaded behavior", Constants.Climber.Down.kP);

        SmartDashboard.putData("update pidf", new InstantCommand(() -> updatePIDF()));
    }

    private void updateSDB() {
        SmartDashboard.putNumber("Climber Position", getPosition());
    }

    private void updatePIDF() {
        this._leftSparkMax.setupPIDF(new PIDFGains(SmartDashboard.getNumber("kP", Constants.Climber.Down.kP), 0, 0,
                SmartDashboard.getNumber("kF", Constants.Climber.Down.kF), 0, 0));
    }

    public void disableInit(){
        this._leftSparkMax.set(0);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        updateSDB();
    }
}
