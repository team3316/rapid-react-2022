package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileSubsystem;
import frc.robot.Constants.ArmConstants;
import frc.robot.utils.LatchedBoolean;

public class Arm extends TrapezoidProfileSubsystem {
    private CANSparkMax _leader;
    private CANSparkMax _follower;
    private SparkMaxLimitSwitch _forwardLimit;
    private SparkMaxLimitSwitch _reverseLimit;
    private LatchedBoolean _forwardState;
    private LatchedBoolean _reverseState;
    private RelativeEncoder _encoder;
    private SparkMaxPIDController _PIDController;
    private ArmFeedforward _feedforward;

    public Arm() {
        super(new TrapezoidProfile.Constraints(
                ArmConstants.maxVelocityDegreesPerSec, ArmConstants.maxAccelerationDegreesPerSecSqrd),
                ArmConstants.startingAngle);

        _leader = new CANSparkMax(ArmConstants.leaderCANID, MotorType.kBrushless);
        _follower = new CANSparkMax(ArmConstants.followerCANID, MotorType.kBrushless);

        _leader.restoreFactoryDefaults();
        _follower.restoreFactoryDefaults();

        _leader.setInverted(ArmConstants.motorInverted);
        _leader.setIdleMode(IdleMode.kBrake);
        _follower.setIdleMode(IdleMode.kBrake);

        _forwardLimit = _leader.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
        _reverseLimit = _leader.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyClosed);
        _forwardLimit.enableLimitSwitch(true);
        _reverseLimit.enableLimitSwitch(true);

        _forwardState = new LatchedBoolean();
        _reverseState = new LatchedBoolean();

        _follower.follow(_leader, true);

        _encoder = _leader.getEncoder();
        _encoder.setPositionConversionFactor(ArmConstants.motorToArmConversionFactor);
        _encoder.setVelocityConversionFactor(ArmConstants.motorToArmConversionFactor / 60);

        _PIDController = _leader.getPIDController();
        setPID();

        setFeedForward();
    }

    private void setPID() {
        _PIDController.setP(ArmConstants.kP, ArmConstants.kPIDSlot);
        _PIDController.setI(0, ArmConstants.kPIDSlot);
        _PIDController.setD(0, ArmConstants.kPIDSlot);
        _PIDController.setIZone(0, ArmConstants.kPIDSlot);
        _PIDController.setFF(0, ArmConstants.kPIDSlot);

        double kMaxOutput = ArmConstants.kMaxOutput;
        _PIDController.setOutputRange(-kMaxOutput, kMaxOutput, ArmConstants.kPIDSlot);
    }

    private void setFeedForward() {
        _feedforward = new ArmFeedforward(0,
                ArmConstants.gravityFF,
                ArmConstants.velocityFF);
    }

    public void setActiveGoal(double angle) {
        System.out.println(angle);
        this.enable();
        super.setGoal(angle);
    }

    @Override
    public void useState(TrapezoidProfile.State state) {
        if (_forwardState.update(_forwardLimit.isPressed())) {
            _encoder.setPosition(ArmConstants.intakeAngle);
        } else if (_reverseState.update(_reverseLimit.isPressed())) {
            _encoder.setPosition(ArmConstants.shootAngle);
        }
        SmartDashboard.putBoolean("down", _forwardLimit.isPressed());
        SmartDashboard.putBoolean("up", _reverseLimit.isPressed());
        SmartDashboard.putNumber("statepositiontobe", state.position);
        SmartDashboard.putNumber("position i think i am", _encoder.getPosition());

        double feedforward = _feedforward.calculate(Math.toRadians(state.position), state.velocity);

        _PIDController.setReference(state.position, ControlType.kPosition, ArmConstants.kPIDSlot, feedforward,
                ArbFFUnits.kPercentOut);

    }
}