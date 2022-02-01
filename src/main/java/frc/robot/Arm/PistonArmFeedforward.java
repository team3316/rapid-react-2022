package frc.robot.Arm;

import java.util.function.DoubleSupplier;

import frc.robot.Arm.subsystems.Piston;
import frc.robot.Constants.ArmConstants;
import frc.robot.motors.units.PositionUnit;

public class PistonArmFeedforward {
    private final double _ks;
    private final double _kt;
    private final double _kv;
    private final double _ka;
    private final Piston _piston;
    private DoubleSupplier _xAcc;
    private DoubleSupplier _zAcc;
       
    public PistonArmFeedforward(double ks, double kt, double kv, double ka, Piston piston) {
        _ks = ks;
        _kt = kt;
        _kv = kv;
        _ka = ka;
        _piston = piston;
    }
      
    /**
    * Creates a new ArmFeedforward with the specified gains. Acceleration gain is defaulted to zero.
    * Units of the gain values will dictate units of the computed feedforward.
    *
    * @param ks The static gain.
    * @param kcos The gravity gain.
    * @param kv The velocity gain.
    */
    public PistonArmFeedforward(double ks, double kcos, double kv, Piston piston) {
        this(ks, kcos, kv, 0, piston);
    }
    public void SetSuppliers(DoubleSupplier xAcc,DoubleSupplier zAcc) {
        _xAcc = xAcc;
        _zAcc = zAcc;
    }
      
    /**
    * Calculates the feedforward from the gains and setpoints.
    *
    * @param positionRadians The position (angle) setpoint.
    * @param velocityRadPerSec The velocity setpoint.
    * @param accelRadPerSecSquared The acceleration setpoint.
    * @return The computed feedforward.
    */
    public double calculate(
        double positionRadians, double velocityRadPerSec, double accelRadPerSecSquared) {
        return _ks * Math.signum(velocityRadPerSec)
                + _kt * getTorque(positionRadians)
                + _kv * velocityRadPerSec
                + _ka * accelRadPerSecSquared;
    }
    private double getTorque(double radAngle) {
        return getAccTorque(radAngle)+_piston.getTourqe(radAngle/Math.PI, PositionUnit.Rotations);
    }
    private double getAccTorque(double radAngle) {
        double radAngleOfForce = Math.PI/2+Math.atan(_xAcc.getAsDouble()/_zAcc.getAsDouble())-radAngle;
        double forceOfForce = 9.81*Math.hypot(_xAcc.getAsDouble(), _zAcc.getAsDouble())*ArmConstants.mass;
        return forceOfForce*Math.cos(radAngleOfForce)*ArmConstants.centerOfMass;
    }
        
}
