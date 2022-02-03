package frc.robot.Arm.subsystems;


import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxLimitSwitch;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileSubsystem;
import frc.robot.Constants.ArmConstants;

public class Arm extends TrapezoidProfileSubsystem {
    private CANSparkMax _leader;
    private CANSparkMax _follower;
    private SparkMaxLimitSwitch _forwardLimit;
    private SparkMaxLimitSwitch _reverseLimit;
    

    public Arm() {
        super(
            new TrapezoidProfile.Constraints(
               ArmConstants.maxVelocityDegreesPerSec, ArmConstants.maxAccelerationRotPerSecSqrd),
            ArmConstants.startingAngle
        );

        _leader = new CANSparkMax(ArmConstants.leaderCANID, MotorType.kBrushless);
        _follower = new CANSparkMax(ArmConstants.followerCANID, MotorType.kBrushless);
        
        _leader.restoreFactoryDefaults();
        _follower.restoreFactoryDefaults();

        _leader.setInverted(ArmConstants.motorInverted);
        _leader.setIdleMode(IdleMode.kBrake);

        _forwardLimit = _leader.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
        _reverseLimit = _leader.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
        _forwardLimit.enableLimitSwitch(true);
        _reverseLimit.enableLimitSwitch(true);

        _follower.follow(_leader, true);
}       

    @Override
    public void useState(TrapezoidProfile.State state) {
    }

    public void updateSDB() {
        SmartDashboard.putBoolean("Forward Limit Enabled", _forwardLimit.isLimitSwitchEnabled());
        SmartDashboard.putBoolean("Reverse Limit Enabled", _reverseLimit.isLimitSwitchEnabled());
    }

    public void updateFromSDB() {
    }

}