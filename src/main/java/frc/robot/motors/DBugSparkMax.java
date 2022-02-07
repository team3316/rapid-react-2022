package frc.robot.motors;

import java.util.Map;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import frc.robot.motors.units.PositionUnit;
import frc.robot.motors.units.UnitConversions;
import frc.robot.motors.units.VelocityUnit;

public class DBugSparkMax extends CANSparkMax implements IDBugMotorController {
    private static final Map<ControlMode, ControlType> controlModeMap = Map.of(
            ControlMode.Current, ControlType.kCurrent,
            ControlMode.Position, ControlType.kPosition,
            ControlMode.Velocity, ControlType.kVelocity);

    private CANSparkMax _leader; // needed in case setInverted is called before follow
    private SparkMaxPIDController _pidController;
    private RelativeEncoder _encoder;
    private final UnitConversions conversions;

    public DBugSparkMax(int deviceNumber, UnitConversions conversions) {
        this(deviceNumber, conversions, MotorType.kBrushless);
    }

    public DBugSparkMax(int deviceId, UnitConversions conversions, MotorType type) {
        super(deviceId, type);

        this.conversions = conversions;
        this._encoder = this.getEncoder();
        this._pidController = this.getPIDController();
    }

    @Override
    public void follow(IDBugMotorController leader) {
        if (leader instanceof CANSparkMax) {
            this._leader = (DBugSparkMax) leader;
            this.follow(_leader, this.getInverted());
        } else {
            throw new IllegalArgumentException("Leader must be a SparkMax");
        }

    }

    @Override
    public void set(ControlMode mode, double value) {
        if (mode == ControlMode.PercentOutput) { // there is no equivalent in ControlType
            super.set(value);
        } else {
            this._pidController.setReference(value, controlModeMap.get(mode));
        }
    }

    @Override
    public void setupPIDF(PIDFGains gains) {
        this._pidController.setP(gains.kP);
        this._pidController.setI(gains.kI);
        this._pidController.setD(gains.kD);
        this._pidController.setFF(gains.kF);
        this._pidController.setIZone(gains.iZone);
        this._pidController.setOutputRange(-1.0, 1.0);
    }

    @Override
    public void setPosition(double value) {
        this._encoder.setPosition(value);
    }

    @Override
    public double getVelocity(VelocityUnit unit) {
        return this._encoder.getVelocity() * conversions.getRPMModifier(unit);

    }

    @Override
    public double getPosition(PositionUnit unit) {
        return this._encoder.getPosition() * conversions.getRotationsModifier(unit);
    }

    @Override
    public void setInverted(boolean inverted) {
        if (this.isFollower())
            this.follow(_leader, inverted);
        else
            super.setInverted(inverted);
    }

    @Override
    public void set(ControlMode mode, double value, VelocityUnit unit) {
        this.set(mode, value / conversions.getRPMModifier(unit));
    }

    @Override
    public void set(ControlMode mode, double value, PositionUnit unit) {
        this.set(mode, value / conversions.getRotationsModifier(unit));
    }
}
