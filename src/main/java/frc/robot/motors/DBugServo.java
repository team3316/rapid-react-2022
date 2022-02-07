package frc.robot.motors;

import java.util.ArrayList;
import java.util.List;

import frc.robot.motors.units.PositionUnit;
import frc.robot.motors.units.UnitConversions;
import frc.robot.motors.units.VelocityUnit;

public class DBugServo extends BetterServo implements IDBugMotorController {

    private List<DBugServo> _followers;
    private DBugServo _leader;
    private final UnitConversions conversions;
    private boolean _inverted;

    public DBugServo(int channel, UnitConversions conversions) {
        super(channel);
        this.conversions = conversions;
        this._followers = new ArrayList<DBugServo>();
    }

    @Override
    public void setInverted(boolean inverted) {
        _inverted = inverted;
    }

    @Override
    public void follow(IDBugMotorController leader) {
        if (leader == null) { // remove leader
            if (this._leader != null) {
                this._leader._followers.remove(this);
                this._leader = null;
            }
        } else if (leader instanceof DBugServo) {
            if (((DBugServo) leader).isFollowing(this)) {
                throw new IllegalArgumentException("Leader is already following follower");
            } else {
                if (_leader != null) {
                    _leader._followers.remove(this); // Cant follow more than one device
                }
                _leader = (DBugServo) leader;
                _leader.addFollower(this);
            }
        } else {
            throw new IllegalArgumentException("Leader must be a DBugServo");
        }
    }

    @Override
    public void set(ControlMode mode, double value) {
        if (mode != ControlMode.Position)
            throw new IllegalArgumentException("Servo only supports Position control mode");

        this.setAngle(_inverted ? super.kMaxServoAngle - (value * 360) : (value * 360));

        for (DBugServo servo : _followers) {
            servo.set(mode, value);
        }
    }

    @Override
    public void setupPIDF(PIDFGains gains) {
        throw new UnsupportedOperationException("Servo does not support this method");
    }

    @Override
    public double getVelocity(VelocityUnit unit) {
        throw new UnsupportedOperationException("Servo does not support this method");
    }

    @Override
    public double getPosition(PositionUnit unit) {
        return this.getAngle() / 360 * conversions.getRotationsModifier(unit);
    }

    private void addFollower(DBugServo follower) {
        this._followers.add(follower);
    }

    private boolean isFollowing(DBugServo leader) {
        for (DBugServo servo : leader._followers) {
            return servo == this || servo.isFollowing(leader);
        }
        return this == leader;
    }

    @Override
    public void set(ControlMode mode, double value, VelocityUnit unit) {
        throw new IllegalArgumentException("Servo only supports Position control mode");
    }

    @Override
    public void set(ControlMode mode, double value, PositionUnit unit) {
        this.set(mode, value / conversions.getRotationsModifier(unit));
    }
}
