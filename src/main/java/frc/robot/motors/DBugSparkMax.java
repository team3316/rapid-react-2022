package frc.robot.motors;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

public class DBugSparkMax extends CANSparkMax {
    private SparkMaxPIDController _pidController;
    private RelativeEncoder _encoder;

    public DBugSparkMax(int deviceNumber) {
        super(deviceNumber, MotorType.kBrushless);

        this._encoder = this.getEncoder();

        this._pidController = this.getPIDController();
    }

    public void setReference(double value, ControlType ctrl) {
        this._pidController.setReference(value, ctrl);
    }

    public void setConversionFactors(double positionFactor, double velocityFactor) {
        this._encoder.setPositionConversionFactor(positionFactor);
        this._encoder.setVelocityConversionFactor(velocityFactor);
    }

    public void setupPIDF(PIDFGains gains) {
        this._pidController.setP(gains.kP);
        this._pidController.setI(gains.kI);
        this._pidController.setD(gains.kD);
        this._pidController.setFF(gains.kF);
        this._pidController.setIZone(gains.iZone);
        this._pidController.setOutputRange(-1.0, 1.0);
    }

    public void setPosition(double value) {
        this._encoder.setPosition(value);
    }

    public double getVelocity() {
        return this._encoder.getVelocity();

    }

    public double getPosition() {
        return this._encoder.getPosition();
    }
}
