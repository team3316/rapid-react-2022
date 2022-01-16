package frc.robot.motors;

import java.util.Map;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

public class DBugSparkMax extends CANSparkMax implements IDBugMotor {
    private static final Map<ControlMode,ControlType> controlModeMap = Map.of(
        ControlMode.Current,ControlType.kCurrent,
        ControlMode.Position, ControlType.kPosition,
        ControlMode.Velocity, ControlType.kVelocity
    );


    private CANSparkMax _leader; // needed in case setInverted is called before follow
    private SparkMaxPIDController _pidController;
    private RelativeEncoder _encoder;
    private final UnitConversions conversions;

    public DBugSparkMax(int deviceId, UnitConversions conversions,MotorType type) {
        super(deviceId, type);

        this.conversions = conversions;
        this._encoder = this.getEncoder();
        this._pidController = this.getPIDController();
    }

    public DBugSparkMax(int deviceNumber, UnitConversions conversions) {
        this(deviceNumber, conversions, MotorType.kBrushless);
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
        if(mode == ControlMode.PercentOutput) { // there is no equivalent in ControlType
            super.set(value);
        } else {
            _pidController.setReference(value,controlModeMap.get(mode));
        }
    }
    
    @Override
    public void setupPIDF(PIDFGains gains) {
        this._pidController.setP(gains.kP);
        this._pidController.setI(gains.kI);
        this._pidController.setD(gains.kD);
        this._pidController.setFF(gains.kF);
        this._pidController.setIZone(gains.iZone);
        // TODO: find tolerance method
        this._pidController.setOutputRange(-1.0, 1.0);
    }

    @Override
    public void setPosition(double value) {
        _encoder.setPosition(value);
    }


    @Override
    public double getVelocity(VelocityUnit unit) {
        return conversions.rpmApplyModifier(_encoder.getVelocity(), unit);
        
    }

    @Override
    public double getPosition(PositionUnit unit) {
        return conversions.rotationsApplyModifier(_encoder.getPosition(), unit);        
    }

    @Override
    public void setInverted(boolean inverted) {
        if(this.isFollower())
            this.follow(_leader, inverted);
        else
            super.setInverted(inverted);
    }
}
