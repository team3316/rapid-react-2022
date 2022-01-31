package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileSubsystem;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.motors.ControlMode;
import frc.robot.motors.DBugSparkMax;
import frc.robot.motors.IDBugMotorController;
import frc.robot.motors.units.PositionUnit;
import frc.robot.motors.units.UnitConversions;

public class Arm extends TrapezoidProfileSubsystem{
    private IDBugMotorController _leaderSM;
    private IDBugMotorController _followerSM;
    private final ArmFeedforward _feedforward = 
        new ArmFeedforward(ArmConstants.staticFF, 
                           ArmConstants.gravityFF, 
                           ArmConstants.velocityFF,
                           ArmConstants.accelerationFF);

    public enum ArmState {
        
        INTAKE(Constants.ArmConstants.intakeAngle),
        SHOOT(Constants.ArmConstants.shootAngle);

        private double _rotations;
        private ArmState(double rotations) {
            _rotations = rotations;
    }
        public double getRotations() {
            return this._rotations;
        }
    }
/** Create a new ArmSubsystem. */
    public Arm() {
        super(
            new TrapezoidProfile.Constraints(
               ArmConstants.maxVelocityRadPerSec, ArmConstants.maxAccelerationRadPerSecSqrd),
            ArmConstants.startingRad
        );

        _leaderSM = new DBugSparkMax(ArmConstants.leaderSMID, new UnitConversions(1/ArmConstants.gearRatioNeoToArm));
        _followerSM = new DBugSparkMax(ArmConstants.followerSMID, new UnitConversions(1/ArmConstants.gearRatioNeoToArm));
        ((CANSparkMax)_followerSM).follow((CANSparkMax)_leaderSM,true);
        ((CANSparkMax)_followerSM).setIdleMode(IdleMode.kBrake);
        ((CANSparkMax)_leaderSM).setIdleMode(IdleMode.kBrake);
        _leaderSM.setupPIDF(ArmConstants.armPID);
        _leaderSM.setPosition(ArmConstants.startingRad);
}

    @Override
    public void useState(TrapezoidProfile.State setpoint) {
         // Calculate the feedforward from the sepoint
        double tempFeedforward = _feedforward.calculate(setpoint.position, setpoint.velocity);
        // Add the feedforward to the PID output to get the motor output
        ((CANSparkMax)_leaderSM).getPIDController().setReference(setpoint.position,CANSparkMax.ControlType.kPosition,0,tempFeedforward);
    }
    
    public void testState(double prcnt) {
        _leaderSM.set(ControlMode.PercentOutput, prcnt);
    }
    public boolean reachedEnd(double goal) {
        return (_leaderSM.getPosition(PositionUnit.Rotations) == goal);
    }
}
