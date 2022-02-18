// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {

    private CANSparkMax _leftSparkMax, _rightSparkMax;
    private SparkMaxPIDController _leftController, _rightController;
    private RelativeEncoder _leftEncoder, _rightEncoder;
    private SparkMaxLimitSwitch _leftHall, _rightHall;

    private static void configurePidController(SparkMaxPIDController controller) {
        controller.setP(Constants.Climber.Down.kP, Constants.Climber.Down.PIDslot);
        controller.setI(0, Constants.Climber.Down.PIDslot);
        controller.setD(0, Constants.Climber.Down.PIDslot);
        controller.setFF(Constants.Climber.Down.kF, Constants.Climber.Down.PIDslot);
        controller.setOutputRange(-Constants.Climber.Down.outputRange, Constants.Climber.Down.outputRange,
                Constants.Climber.Down.PIDslot);

        controller.setP(Constants.Climber.Up.kP, Constants.Climber.Up.PIDslot);
        controller.setI(0, Constants.Climber.Up.PIDslot);
        controller.setD(0, Constants.Climber.Up.PIDslot);
        controller.setFF(Constants.Climber.Up.kF, Constants.Climber.Up.PIDslot);
        controller.setOutputRange(-Constants.Climber.Up.outputRange, Constants.Climber.Up.outputRange,
                Constants.Climber.Up.PIDslot);

    }

    private static void configureSparkMax(CANSparkMax sparkMax) {
        sparkMax.restoreFactoryDefaults();
        sparkMax.setIdleMode(IdleMode.kBrake);

        sparkMax.enableSoftLimit(SoftLimitDirection.kForward, true);
        sparkMax.enableSoftLimit(SoftLimitDirection.kReverse, true);
        sparkMax.setSoftLimit(SoftLimitDirection.kForward, (float) Constants.Climber.climbExtentionHeight);
        sparkMax.setSoftLimit(SoftLimitDirection.kReverse, (float) Constants.Climber.startingPosition);
    }

    private static void configureEncoder(RelativeEncoder encoder) {
        encoder.setPositionConversionFactor(Constants.Climber.conversionFactor);
        encoder.setVelocityConversionFactor(Constants.Climber.conversionFactor / 60);
        encoder.setPosition(Constants.Climber.startingPosition);
    }

    public Climber() {
        this._leftSparkMax = new CANSparkMax(Constants.Climber.leftID, MotorType.kBrushless);
        this._rightSparkMax = new CANSparkMax(Constants.Climber.rightID, MotorType.kBrushless);
        configureSparkMax(_leftSparkMax);
        configureSparkMax(_rightSparkMax);

        this._leftController = _leftSparkMax.getPIDController();
        this._rightController = _rightSparkMax.getPIDController();
        configurePidController(_leftController);
        configurePidController(_rightController);

        this._leftEncoder = _leftSparkMax.getEncoder();
        this._rightEncoder = _rightSparkMax.getEncoder();
        configureEncoder(_leftEncoder);
        configureEncoder(_rightEncoder);

        this._rightSparkMax.setInverted(false);

        this._leftHall = this._leftSparkMax.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyClosed);
        this._rightHall = this._rightSparkMax.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyClosed);
        this._leftHall.enableLimitSwitch(true);
        this._rightHall.enableLimitSwitch(true);

        initSDB();
    }

    public void setVelocity(double value, int slot) {
        // TODO: Replace this with Aux position control
        this._leftController.setReference(value, ControlType.kVelocity, slot);
        this._rightController.setReference(value, ControlType.kVelocity, slot);
    }

    public void set(double value) {
        _leftSparkMax.set(value);
        _rightSparkMax.set(value);
    }

    public void setL(double value) {
        _leftSparkMax.set(value);
    }

    public void setR(double value) {
        _rightSparkMax.set(value);
    }

    private double getLeftPosition() {
        return this._leftSparkMax.getEncoder().getPosition();
    }

    private double getRightPosition() {
        return this._rightSparkMax.getEncoder().getPosition();
    }

    private void initSDB() {
        SmartDashboard.setDefaultNumber("kP down position", Constants.Climber.Down.kP);
        SmartDashboard.setDefaultNumber("kF down position", Constants.Climber.Down.kF);
        SmartDashboard.setDefaultNumber("kP up position", Constants.Climber.Up.kP);
        SmartDashboard.setDefaultNumber("kF up position", Constants.Climber.Up.kF);

        SmartDashboard.setDefaultNumber("output range down", Constants.Climber.Down.outputRange);
        SmartDashboard.setDefaultNumber("output range up", Constants.Climber.Up.outputRange);

        SmartDashboard.setDefaultNumber("soft limit forward", Constants.Climber.climbExtentionHeight);
        SmartDashboard.setDefaultNumber("soft limit reverse", Constants.Climber.startingPosition);

        SmartDashboard.putData("update from SDB", new InstantCommand(() -> updateFromSDB()));

        SmartDashboard.putData("enable left soft limit", new InstantCommand(() -> enableSoftLimit(true)));
        SmartDashboard.putData("disable left soft limit", new InstantCommand(() -> enableSoftLimit(false)));
        SmartDashboard.putData("update soft limit position", new InstantCommand(() -> updateSoftLimitPosition()));
    }

    private void enableSoftLimit(boolean enabled) {
        this._leftSparkMax.enableSoftLimit(SoftLimitDirection.kForward, enabled);
        this._leftSparkMax.enableSoftLimit(SoftLimitDirection.kReverse, enabled);
        this._rightSparkMax.enableSoftLimit(SoftLimitDirection.kForward, enabled);
        this._rightSparkMax.enableSoftLimit(SoftLimitDirection.kReverse, enabled);
    }

    private void updateSoftLimitPosition() {
        this._rightSparkMax.setSoftLimit(SoftLimitDirection.kReverse,
                (float) SmartDashboard.getNumber("soft limit reverse", Constants.Climber.startingPosition));
        this._rightSparkMax.setSoftLimit(SoftLimitDirection.kForward,
                (float) SmartDashboard.getNumber("soft limit forward", Constants.Climber.climbExtentionHeight));
        this._leftSparkMax.setSoftLimit(SoftLimitDirection.kReverse,
                (float) SmartDashboard.getNumber("soft limit reverse", Constants.Climber.startingPosition));
        this._leftSparkMax.setSoftLimit(SoftLimitDirection.kForward,
                (float) SmartDashboard.getNumber("soft limit forward", Constants.Climber.climbExtentionHeight));
    }

    private void updateSDB() {
        SmartDashboard.putNumber("Climber Left Position", getLeftPosition());
        SmartDashboard.putNumber("Climber Right Position", getRightPosition());

        // SmartDashboard.putNumber("right encoder pos", SmartDashboard.getNumber("right
        // encoder pos", Constants.Climber.startingPosition));
        // SmartDashboard.putNumber("left encoder pos", SmartDashboard.getNumber("left
        // encoder pos", Constants.Climber.startingPosition));
    }

    public void setEncoderPosition(double position) {
        this._rightEncoder.setPosition(position);
        this._leftEncoder.setPosition(position);
    }

    private void updateFromSDB() {
        updateController(_leftController);
        updateController(_rightController);
    }

    private static void updateController(SparkMaxPIDController controller) {
        controller.setP(SmartDashboard.getNumber("kP down position", Constants.Climber.Down.kP),
                Constants.Climber.Down.PIDslot);
        controller.setFF(SmartDashboard.getNumber("kF down position", Constants.Climber.Down.kF),
                Constants.Climber.Down.PIDslot);
        controller.setOutputRange(-SmartDashboard.getNumber("output range down", Constants.Climber.Down.outputRange),
                SmartDashboard.getNumber("output range down", Constants.Climber.Down.outputRange),
                Constants.Climber.Down.PIDslot);

        controller.setP(SmartDashboard.getNumber("kP up position", Constants.Climber.Up.kP),
                Constants.Climber.Up.PIDslot);
        controller.setFF(SmartDashboard.getNumber("kF up position", Constants.Climber.Up.kF),
                Constants.Climber.Up.PIDslot);
        controller.setOutputRange(-SmartDashboard.getNumber("output range up", Constants.Climber.Up.outputRange),
                SmartDashboard.getNumber("output range up", Constants.Climber.Up.outputRange),
                Constants.Climber.Up.PIDslot);
    }

    public void disableInit() {
        set(0);
    }

    @Override
    public void periodic() {
        if (this._leftHall.isPressed()) {
            this._leftEncoder.setPosition(Constants.Climber.startingPosition);
        }
        if (this._rightHall.isPressed()) {
            this._rightEncoder.setPosition(Constants.Climber.startingPosition);
        }
        // This method will be called once per scheduler run
        updateSDB();
    }
}
