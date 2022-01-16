package frc.robot.motors;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.IMotorController;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.BaseTalon;

public class DBugTalon extends BaseTalon implements IDBugMotor {
    private boolean _isFollower; // needed in case setInverted is called before follow
    private double _velocityConversionFactor;
    private double _positionConversionFactor;

    public DBugTalon(int deviceNumber, TalonModel model, FeedbackDevice sensor) {
        super(deviceNumber, model.toString());
        
        super.configSelectedFeedbackSensor(sensor);
    }

    @Override
    public void setRotationFactors(PositionUnit positionUnit, 
                                   VelocityUnit velocityUnit, 
                                   double gearRatio , 
                                   double wheelDiameterMeters, 
                                   int upr) {
        // Units / Units per Rotation * gearRatio = Rotations of output shaft
        super.configSelectedFeedbackCoefficient(gearRatio / upr);
        switch (positionUnit) {
            case Rotations:
                // already in rotations
                _positionConversionFactor = 1;
                break;
            case Degrees:
                // Rotations * 360 = Angle of output shaft
                _positionConversionFactor =  360;
                break;
            case Meters:
                // Rotations * circumference = Distance traveled by wheel in meters
                _positionConversionFactor = wheelDiameterMeters * Math.PI;
                break;
        }
        switch (velocityUnit) {
            case RPM:
                // Rotations per 100ms * 10  * 60 = Revolutions of output shaft per minute
                _velocityConversionFactor = 10 * 60 ;
                break;
            case MetersPerSecond:
                // Rotations per 100ms * 10 = RPS. RPS * gearRatio * circumference = wheel meters per second
                _velocityConversionFactor = 10 * wheelDiameterMeters * Math.PI;
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
