package frc.robot.Arm.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileSubsystem;
import frc.robot.Constants.ArmConstants;

public class Arm extends TrapezoidProfileSubsystem {
    private CANSparkMax _leader;
    private CANSparkMax _follower;
    private SparkMaxLimitSwitch _forwardLimit;
    private SparkMaxLimitSwitch _reverseLimit;
    private RelativeEncoder _encoder;
    private SparkMaxPIDController _PIDController;

    public Arm() {
        super(
                new TrapezoidProfile.Constraints(
                        ArmConstants.maxVelocityDegreesPerSec, ArmConstants.maxAccelerationRotPerSecSqrd),
                ArmConstants.startingAngle);

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

        _encoder = _leader.getEncoder();
        _encoder.setPositionConversionFactor(ArmConstants.motorToArmConversionFactor);
        _encoder.setVelocityConversionFactor(ArmConstants.motorToArmConversionFactor);

        _PIDController = _leader.getPIDController();
        setPID();

        initSDB();
    }

    private void setPID() {
        _PIDController.setP(SmartDashboard.getNumber("P Gain", ArmConstants.kP));
        _PIDController.setI(0);
        _PIDController.setD(0);
        _PIDController.setIZone(0);
        _PIDController.setFF(0);

        double kMaxOutput = SmartDashboard.getNumber("Max Output", ArmConstants.kMaxOutput);
        _PIDController.setOutputRange(-kMaxOutput, kMaxOutput);
    }

    @Override
    public void useState(TrapezoidProfile.State state) {
    }

    @Override
    public void periodic() {
        super.periodic();

        updateSDB();
    }

    public void initSDB() {
        SmartDashboard.putNumber("P Gain", SmartDashboard.getNumber("P Gain", ArmConstants.kP));
        SmartDashboard.putNumber("Max Output", SmartDashboard.getNumber("Max Output", ArmConstants.kMaxOutput));

        SmartDashboard.putData("Set PID", new InstantCommand(() -> setPID()));
    }

    public void updateSDB() {
        SmartDashboard.putBoolean("Forward Limit Enabled", _forwardLimit.isLimitSwitchEnabled());
        SmartDashboard.putBoolean("Reverse Limit Enabled", _reverseLimit.isLimitSwitchEnabled());

        SmartDashboard.putNumber("Arm Position", _encoder.getPosition());
        SmartDashboard.putNumber("Arm Velocity", _encoder.getVelocity());
    }
}