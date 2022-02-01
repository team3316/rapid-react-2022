package frc.robot.Arm.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.motors.units.PositionUnit;

public class Piston extends SubsystemBase{
    private final double _baseLength;
    private final double _baseHeight;
    private final double _armLength;
    private final double _armHeight;
    private final double _armAngle;
    private final double _baseAngle;
    private final double _armHyp;
    private final double _baseHyp;

    private final double _closedLength;
    private final double _extendLength;
    private final double _forceOfPiston;

    public Piston(double baseLength, double baseHeight, double armLength, double armHeight, double forceOfPiston,double closedLength,double extendLength) {
        _baseLength = baseLength;
        _baseHeight = baseHeight;
        _armLength = armLength;
        _armHeight = armHeight;
        _closedLength = closedLength;
        _extendLength = extendLength;
        _forceOfPiston = forceOfPiston;

        _armHyp = Math.hypot(_armLength, _armHeight);
        _baseHyp = Math.hypot(_baseLength, _baseHeight);

        _armAngle = Math.atan(_armHeight/_armLength);
        _baseAngle = Math.atan(_baseHeight/_baseLength);

    }
    private double getRealAngle(double angle, PositionUnit unit) {
        double radAngle;
        switch (unit) {
            case Rotations:
                radAngle = angle*2*Math.PI;
                break;
            case Degrees:
                radAngle = angle*Math.PI/180;
                break;
            default:
                radAngle = 0;
        }
        return radAngle - _armAngle - _baseAngle;
    }
    private double getPistonLength(double angle, PositionUnit unit) {
        return Math.pow(_armHyp,2)+Math.pow(_baseHyp,2) - 2 * Math.cos(getRealAngle(angle, unit))*_armHyp*_baseHyp;
    }
    private double getTourqeAngle(double pistonLength, double angle, PositionUnit unit) {
        return Math.asin(Math.sin(this.getRealAngle(angle, unit)*_baseHyp/this.getPistonLength(angle, unit)));
    }
    private double lengthToForce(double length) {
        return 0.3*(_forceOfPiston/_extendLength)*(_closedLength+_extendLength-length)+_forceOfPiston;
    }
    public double getTourqe(double angle, PositionUnit unit) {
        double pistonLength = this.getPistonLength(angle, unit);
        return this.lengthToForce(pistonLength)*Math.cos(getTourqeAngle(pistonLength,angle,unit))*_armHyp;
    }

}