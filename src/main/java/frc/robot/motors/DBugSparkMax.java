package frc.robot.motors;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

public class DBugSparkMax extends CANSparkMax implements IDBugMotor {
    private CANSparkMax _leader; // needed in case setInverted is called before follow
    private SparkMaxPIDController _pidController;
    private RelativeEncoder _encoder;

    public DBugSparkMax(int deviceId, MotorType type) {
        super(deviceId, type);

        this._encoder = this.getEncoder();
        this._pidController = this.getPIDController();
    }

    public DBugSparkMax(int deviceNumber) {
        this(deviceNumber, MotorType.kBrushless);
    }

    @Override
    public void setRotationFactors(PositionUnit positionUnit, 
                                   VelocityUnit velocityUnit, 
                                   double gearRatio , 
                                   double wheelDiameterMeters, 
                                   int upr) {
        switch (positionUnit) {
            case Rotations:
                // Rotations * gearRatio = Rotations of output shaft
                _encoder.setPositionConversionFactor(gearRatio);
                break;
            case Degrees:
                // Rotations * gearRatio * 360 = Angle of output shaft
                _encoder.setPositionConversionFactor(gearRatio * 360);
                break;
            case Meters:
                // Rotations * gearRatio * circumference = Distance traveled by wheel in meters
                _encoder.setPositionConversionFactor(gearRatio * wheelDiameterMeters * Math.PI);
                break;
        }
        switch (velocityUnit) {
            case RPM:
                // RPM * gearRatio = Revolutions of output shaft per minute
                _encoder.setVelocityConversionFactor(gearRatio);
                break;
            case MetersPerSecond:
                // RPM / 60 = RPS. RPS * gearRatio * circumference = wheel meters per second
                _encoder.setVelocityConversionFactor(gearRatio * wheelDiameterMeters * Math.PI / 60);
                break;
        
        }
    }

    @Override
    public void follow(IDBugMotor leader) {
        if(leader instanceof CANSparkMax) {
            _leader = (DBugSparkMax) leader;
            this.follow(_leader, this.getInverted());
        } else {
            throw new IllegalArgumentException("Leader must be a SparkMax");
        }
        
    }

    @Override
    public void set(ControlMode mode, double value) {
        switch (mode) {
            case Current:
                _pidController.setReference(value,ControlType.kCurrent);
                break;
            case PercentOutput:
                this.set(value);
                break;
            case Position:
                _pidController.setReference(value,ControlType.kVelocity);
                break;
            case Velocity:
                _pidController.setReference(value,ControlType.kVelocity);
                break;

        }    
    }
    
    @Override
    public void setupPIDF(double kP, double kI, double kD, double kF) {
        this._pidController.setP(kP);
        this._pidController.setI(kI);
        this._pidController.setD(kD);
        this._pidController.setFF(kF);
        this._pidController.setOutputRange(-1.0, 1.0);
    }

    @Override
    public void setPosition(double value) {
        _encoder.setPosition(value);
    }


    @Override
    public double getVelocity() {
        return _encoder.getVelocity();
        
    }

    @Override
    public double getPosition() {
        return _encoder.getPosition();        
    }

    @Override
    public void setInverted(boolean inverted) {
        if(this.isFollower())
            this.follow(_leader, inverted);
        else
            super.setInverted(inverted);
    }
}
