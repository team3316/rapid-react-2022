package frc.robot.motors;

import java.util.List;

import edu.wpi.first.wpilibj.Servo;
import frc.robot.motors.units.PositionUnit;
import frc.robot.motors.units.UnitConversions;
import frc.robot.motors.units.VelocityUnit;

public class DBugServo extends Servo implements IDBugMotorController {

    private List<DBugServo> _followers;
    private final UnitConversions conversions;

    public DBugServo(int channel, UnitConversions conversions) {
        super(channel);
        this.conversions = conversions;
        this._followers = List.of();
    }

    @Override
    public void setInverted(boolean inverted) {
        // We can't invert the servo since we don't know the max and min angles
        throw new UnsupportedOperationException("Servo does not support this method");
    }

    @Override
    public void follow(IDBugMotorController leader) {
        if (leader instanceof DBugServo)
            if (((DBugServo) leader).isFollowing(this)){
                throw new IllegalArgumentException("Leader is already following follower");
            } else {
                ((DBugServo) leader).addFollower(this);
            }
        else {
            throw new IllegalArgumentException("Leader must be a DBugServo");
        }
    }

    @Override
    public void set(ControlMode mode, double value) {
        if (mode != ControlMode.Position)
            throw new IllegalArgumentException("Servo only supports Position control mode");
        
        this.setAngle(value * 360);
        
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
        for (DBugServo servo : this._followers) {
            return servo == leader || servo.isFollowing(leader);
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
