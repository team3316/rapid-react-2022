// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController.AccelStrategy;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {

    private CANSparkMax _leftSparkMax, _rightSparkMax;

    private SparkMaxPIDController _PidController;

    private double _lastPosition;

    public Climber() {
        this._leftSparkMax = new CANSparkMax(Constants.Climber.leftID, MotorType.kBrushless);
        this._rightSparkMax = new CANSparkMax(Constants.Climber.rightID, MotorType.kBrushless);

        this._leftSparkMax.restoreFactoryDefaults();
        this._rightSparkMax.restoreFactoryDefaults();

        this._leftSparkMax.setIdleMode(IdleMode.kBrake);
        this._rightSparkMax.setIdleMode(IdleMode.kBrake);

        this._rightSparkMax.follow(this._leftSparkMax, true);

        this._PidController = _leftSparkMax.getPIDController();

        this._PidController.setP(Constants.Climber.Down.kP, Constants.Climber.Down.PIDslot);
        this._PidController.setI(0, Constants.Climber.Down.PIDslot);
        this._PidController.setD(0, Constants.Climber.Down.PIDslot);
        this._PidController.setFF(Constants.Climber.Down.kF, Constants.Climber.Down.PIDslot);
        this._PidController.setOutputRange(-1.0, 1.0, Constants.Climber.Down.PIDslot);
        
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
        return this._leftSparkMax.getEncoder().getPosition();
    }

    private void initSDB() {
        SmartDashboard.setDefaultNumber("kP down position", Constants.Climber.Down.kP);
        SmartDashboard.setDefaultNumber("kF down position", Constants.Climber.Down.kF);
        SmartDashboard.setDefaultNumber("kP up position", Constants.Climber.Up.kP);

        SmartDashboard.setDefaultNumber("max velocity down", Constants.Climber.Down.maxVelMetersPerSecond);
        SmartDashboard.setDefaultNumber("max velocity up", Constants.Climber.Up.maxVelMetersPerSecond);
        SmartDashboard.setDefaultNumber("min velocity down", Constants.Climber.Down.minVelMetersPerSecond);
        SmartDashboard.setDefaultNumber("min velocity up", Constants.Climber.Up.minVelMetersPerSecond);

        SmartDashboard.setDefaultNumber("max acc down", Constants.Climber.Down.maxAccMetersPerSecondSqrd);
        SmartDashboard.setDefaultNumber("max acc up", Constants.Climber.Up.maxAccMetersPerSecondSqrd);

        SmartDashboard.putData("update from SDB", new InstantCommand(() -> updateFromSDB()));
    }

    private void updateSDB() {
        SmartDashboard.putNumber("Climber Position", getPosition());
    }

    private void updateFromSDB() {
        this._PidController.setP(SmartDashboard.getNumber("kP down position", Constants.Climber.Down.kP), Constants.Climber.Down.PIDslot);
        this._PidController.setFF(SmartDashboard.getNumber("kF down position", Constants.Climber.Down.kF), Constants.Climber.Down.PIDslot);
        this._PidController.setSmartMotionMaxVelocity(SmartDashboard.getNumber("max velocity down", Constants.Climber.Down.maxVelMetersPerSecond), Constants.Climber.Down.PIDslot);
        this._PidController.setSmartMotionMinOutputVelocity(SmartDashboard.getNumber("min velocity down", Constants.Climber.Down.minVelMetersPerSecond), Constants.Climber.Down.PIDslot);
        this._PidController.setSmartMotionMaxAccel(SmartDashboard.getNumber("max acc down", Constants.Climber.Down.maxAccMetersPerSecondSqrd), Constants.Climber.Down.PIDslot);

        this._PidController.setP(SmartDashboard.getNumber("kP up position", Constants.Climber.Up.kP), Constants.Climber.Up.PIDslot);
        this._PidController.setFF(SmartDashboard.getNumber("kF up position", Constants.Climber.Up.kF), Constants.Climber.Up.PIDslot);
        this._PidController.setSmartMotionMaxVelocity(SmartDashboard.getNumber("max velocity up", Constants.Climber.Up.maxVelMetersPerSecond), Constants.Climber.Up.PIDslot);
        this._PidController.setSmartMotionMinOutputVelocity(SmartDashboard.getNumber("min velocity up", Constants.Climber.Up.minVelMetersPerSecond), Constants.Climber.Up.PIDslot);
        this._PidController.setSmartMotionMaxAccel(SmartDashboard.getNumber("max acc up", Constants.Climber.Up.maxAccMetersPerSecondSqrd), Constants.Climber.Up.PIDslot);
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
