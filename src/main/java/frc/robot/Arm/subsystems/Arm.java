package frc.robot.Arm.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.SparkMaxLimitSwitch;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileSubsystem;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.motors.DBugSparkMax;
import frc.robot.motors.units.PositionUnit;
import frc.robot.motors.units.UnitConversions;

public class Arm extends TrapezoidProfileSubsystem{
    private DBugSparkMax _leaderSM;
    private DBugSparkMax _followerSM;
    private SparkMaxLimitSwitch _forwardLimit;
    private SparkMaxLimitSwitch _reverseLimit;
    private ArmFeedforward _feedforward = 
        new ArmFeedforward(ArmConstants.staticFF, 
                           ArmConstants.gravityFF, 
                           ArmConstants.velocityFF,
                           ArmConstants.accelerationFF);
                           //new Piston(ArmConstants.baseLength, ArmConstants.baseHeight, ArmConstants.armLength, ArmConstants.armHeight, ArmConstants.forceOfPiston, ArmConstants.closedLength, ArmConstants.extendLength));

    public enum ArmState {
        // TODO: Calibrate angles
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
    public Arm(DoubleSupplier xAcc,DoubleSupplier zAcc) {
        super(
            new TrapezoidProfile.Constraints(
                // TODO: Add to SDB and add to same update function.
               ArmConstants.maxVelocityRadPerSec, ArmConstants.maxAccelerationRadPerSecSqrd),
            // TODO: Calibrate
            ArmConstants.startingRad
        );


        _leaderSM = new DBugSparkMax(ArmConstants.leaderSMID, new UnitConversions(1/ArmConstants.gearRatioNeoToArm));
        _followerSM = new DBugSparkMax(ArmConstants.followerSMID, new UnitConversions(1/ArmConstants.gearRatioNeoToArm));
        _followerSM.follow((CANSparkMax)_leaderSM, true);
        _followerSM.setIdleMode(IdleMode.kBrake);
        _leaderSM.setIdleMode(IdleMode.kBrake);
        _leaderSM.setupPIDF(ArmConstants.armPID);
        _leaderSM.setPosition(ArmConstants.startingRad);
        //TODO calibrate which is forward who is reverse
        _forwardLimit = _leaderSM.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
        _reverseLimit = _leaderSM.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
        _forwardLimit.enableLimitSwitch(true);
        _reverseLimit.enableLimitSwitch(true);
}

    @Override
    public void useState(TrapezoidProfile.State setpoint) {
         // Calculate the feedforward from the sepoint
        double tempFeedforward = _feedforward.calculate(setpoint.position, setpoint.velocity,0);
        // Add the feedforward to the PID output to get the motor output
        _leaderSM.getPIDController().setReference(setpoint.position, CANSparkMax.ControlType.kPosition, 0, tempFeedforward);
    }
    public boolean reachedEnd(double goal) {
        return (_leaderSM.getPosition(PositionUnit.Rotations) == goal);
    }
    public void UpdateFromSDB(double Ks, double Kg, double Kv, double Ka) {
        _feedforward = new ArmFeedforward(SmartDashboard.getNumber("static gain", ArmConstants.staticFF),
                                    SmartDashboard.getNumber("gravity gain", ArmConstants.gravityFF),
                                    SmartDashboard.getNumber("speed gain", ArmConstants.velocityFF),
                                    SmartDashboard.getNumber("acc gain", ArmConstants.accelerationFF));
        
    }
}
