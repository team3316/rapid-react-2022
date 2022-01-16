package frc.robot.motors;

import java.util.Map;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.IMotorController;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.BaseTalon;

public class DBugTalon extends BaseTalon implements IDBugMotor {
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
    public void follow(IDBugMotor leader) {
        if(leader instanceof IMotorController) {
            _isFollower = true;
            super.follow((IMotorController)leader);

            // Make sure we have proper inversion if setInverted was called
            // TODO Check if this works as expected
            this.setInverted(super.getInverted());
        } else {
            throw new IllegalArgumentException("Leader must be a CTRE motor controller");
        }
        
    }

    @Override
    public void set(ControlMode mode, double value) {
        super.set(controlModeMap.get(mode),value);
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

    @Override
    public double getVelocity(VelocityUnit unit) {
        return conversions.rpmApplyModifier(this.getSelectedSensorVelocity()/conversions.upr * 10 * 60, unit);
    }

    @Override
    public double getPosition(PositionUnit unit) {
        return conversions.rotationsApplyModifier(this.getSelectedSensorPosition()/conversions.upr, unit);
    }

    @Override
    public void setInverted(boolean inverted) {
        if(_isFollower)
            super.setInverted(inverted ? InvertType.OpposeMaster : InvertType.FollowMaster);
        else
            super.setInverted(inverted);
    }
   
}
