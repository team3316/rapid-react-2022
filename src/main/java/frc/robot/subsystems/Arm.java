package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileSubsystem;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.motors.DBugSparkMax;
import frc.robot.motors.IDBugMotor;

public class Arm extends TrapezoidProfileSubsystem{
    private IDBugMotor leaderSM;
    private IDBugMotor followerSM;
    private final ArmFeedforward feedforward = 
        new ArmFeedforward(ArmConstants.staticFF, 
                           ArmConstants.gravityFF, 
                           ArmConstants.velocityFF,
                           ArmConstants.accelerationFF);

    public enum armState {
        
        INTAKE(Constants.ArmConstants.intakeAngle),
        SHOOT(Constants.ArmConstants.shootAngle);

        private double _angle;
        private armState(double angle) {
            _angle = angle;
    }
        public double getAngle() {
            return this._angle;
        }
    }
/** Create a new ArmSubsystem. */
    public Arm() {
        super(
            new TrapezoidProfile.Constraints(
               ArmConstants.maxVelocityRadPerSec, ArmConstants.maxAccelerationRadPerSecSqrd),
            ArmConstants.startingRad
        );

        leaderSM = new DBugSparkMax(ArmConstants.leaderSMID);
        followerSM = new DBugSparkMax(ArmConstants.followerSMID);
        followerSM.follow(leaderSM);
        followerSM.setInverted(true);
        leaderSM.setupPIDF(ArmConstants.armPID);
        leaderSM.setPosition(ArmConstants.startingRad);
}

    @Override
    public void useState(TrapezoidProfile.State setpoint) {
         // Calculate the feedforward from the sepoint
        double tempFeedforward = feedforward.calculate(setpoint.position, setpoint.velocity);
        // Add the feedforward to the PID output to get the motor output
        ((CANSparkMax)leaderSM).getPIDController().setReference(setpoint.position,CANSparkMax.ControlType.kPosition,0,tempFeedforward);
    }
    
}
