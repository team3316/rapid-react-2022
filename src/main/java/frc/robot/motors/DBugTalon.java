package frc.robot.motors;

import java.util.Map;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.IMotorController;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.BaseTalon;

import frc.robot.motors.units.PositionUnit;
import frc.robot.motors.units.UnitConversions;
import frc.robot.motors.units.VelocityUnit;

public class DBugTalon extends BaseTalon implements IDBugMotorController {
    private final Map<ControlMode,com.ctre.phoenix.motorcontrol.ControlMode> controlModeMap = 
    Map.of(
        ControlMode.Current,com.ctre.phoenix.motorcontrol.ControlMode.Current,
        ControlMode.PercentOutput,com.ctre.phoenix.motorcontrol.ControlMode.PercentOutput,
        ControlMode.Position,com.ctre.phoenix.motorcontrol.ControlMode.Position,
        ControlMode.Velocity,com.ctre.phoenix.motorcontrol.ControlMode.Velocity
    ); 

    private boolean _isFollower; // needed in case setInverted is called before follow
    private final UnitConversions conversions;

    public DBugTalon(int deviceNumber, TalonModel model, UnitConversions conversions, FeedbackDevice sensor) {
        super(deviceNumber, model.name);

        this.conversions = conversions;
        super.configSelectedFeedbackSensor(sensor);
    }


    @Override
    public void follow(IDBugMotorController leader) {
        if(leader instanceof IMotorController) {
            this._isFollower = true;
            super.follow((IMotorController)leader);

            // Make sure we have proper inversion if setInverted was called
            // TODO Check if this works as expected
            this.setInverted(super.getInverted());
        } else {
            throw new IllegalArgumentException("Leader must be a CTRE motor controller");
        }
        
    }

    public void setupPIDF(PIDFGains gains) {
        this.selectProfileSlot(0,0);
        this.config_kP(0, gains.kP);
        this.config_kI(0, gains.kI);
        this.config_kD(0, gains.kD);
        this.config_kF(0, gains.kF);
        this.config_IntegralZone(0, gains.iZone);
        this.configAllowableClosedloopError(0, gains.tolerance);
    }

    @Override
    public void setPosition(double value) {
        this.setSelectedSensorPosition(value);
    }

    private double getVelocityModifier(VelocityUnit unit) {
        return 10 * 60 * conversions.getRPMModifier(unit) / conversions.upr;
    }

    private double getPositionModifier(PositionUnit unit) {
        return conversions.getRotationsModifier(unit) / conversions.upr;
    }

    @Override
    public double getVelocity(VelocityUnit unit) {
        return this.getSelectedSensorVelocity() * this.getVelocityModifier(unit);
    }

    @Override
    public double getPosition(PositionUnit unit) {
        return this.getSelectedSensorPosition() * this.getPositionModifier(unit);
    }

    @Override
    public void setInverted(boolean inverted) {
        if(this._isFollower)
            super.setInverted(inverted ? InvertType.OpposeMaster : InvertType.FollowMaster);
        else
            super.setInverted(inverted);
    }


    
    public void set(ControlMode mode, double value) {
        super.set(controlModeMap.get(mode),value);
    }

    @Override
    public void set(ControlMode mode, double value, VelocityUnit unit) {
        this.set(mode, value / this.getVelocityModifier(unit));
    }


    @Override
    public void set(ControlMode mode, double value, PositionUnit unit) {
        this.set(mode, value / this.getPositionModifier(unit));        
    }
   
}
