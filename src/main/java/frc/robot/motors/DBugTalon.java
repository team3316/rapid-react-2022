package frc.robot.motors;

import com.ctre.phoenix.motorcontrol.IMotorController;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.BaseTalon;

public class DBugTalon extends BaseTalon implements IDBugMotor {

    private boolean _isFollower; // needed in case setInverted is called before follow
    private double _velocityConversionFactor;
    private double _positionConversionFactor;

    public DBugTalon(int deviceNumber, TalonModel model) {
        super(deviceNumber, model.toString());
        
    }

    @Override
    public void setRotationFactors(PositionUnit positionUnit, 
                                   VelocityUnit velocityUnit, 
                                   double gearRatio , 
                                   double wheelDiameterMeters, 
                                   int upr) {
        switch (positionUnit) {
            case Rotations:
                // Units / Units per Rotation * gearRatio = Rotations of output shaft
                _positionConversionFactor = gearRatio / upr;
                break;
            case Degrees:
                // Units / Units per Rotation * gearRatio * 360 = Angle of output shaft
                _positionConversionFactor = gearRatio / upr * 360;
                break;
            case Meters:
                // Units / Units per Rotation * gearRatio * circumference = Distance traveled by wheel in meters
                _positionConversionFactor = gearRatio * wheelDiameterMeters * Math.PI / upr;
                break;
        }
        switch (velocityUnit) {
            case RPM:
                // Units per 100ms * 10 / upr = RPS, RPS * 60 * gearRatio = Revolutions of output shaft per minute
                _velocityConversionFactor = 10 / upr * 60 * gearRatio;
                break;
            case MetersPerSecond:
                // Units per 100ms * 10 / upr = RPS. RPS * gearRatio * circumference = wheel meters per second
                _velocityConversionFactor = 10 / upr * gearRatio * wheelDiameterMeters * Math.PI;
                break;
        
        }
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
        switch (mode) {
            case Current:
                super.set(com.ctre.phoenix.motorcontrol.ControlMode.Current, value);
                break;
            case PercentOutput:
                super.set(com.ctre.phoenix.motorcontrol.ControlMode.PercentOutput, value);
                break;
            case Position:
                super.set(com.ctre.phoenix.motorcontrol.ControlMode.Position, value);
                break;
            case Velocity:
                super.set(com.ctre.phoenix.motorcontrol.ControlMode.Velocity, value);
                break;

        }    
        
    }

    public void setupPIDF(double kP, double kI, double kD, double kF) {
        this.selectProfileSlot(0,0);
        this.config_kP(0, kP);
        this.config_kI(0, kI);
        this.config_kD(0, kD);
        this.config_kF(0, kF);
    }

    @Override
    public void setPosition(double value) {
        this.setSelectedSensorPosition(value);
    }

    @Override
    public double getVelocity() {
        return this.getSelectedSensorVelocity() * _velocityConversionFactor;
    }

    @Override
    public double getPosition() {
        return this.getSelectedSensorPosition() * _positionConversionFactor;
    }

    @Override
    public void setInverted(boolean inverted) {
        if(_isFollower)
            super.setInverted(inverted ? InvertType.OpposeMaster : InvertType.FollowMaster);
        else
            super.setInverted(inverted);
    }
   
}
