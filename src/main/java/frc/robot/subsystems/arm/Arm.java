package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileCommand;
import frc.robot.Constants.ArmConstants;
import frc.robot.motors.DBugSparkMax;
import frc.robot.utils.LatchedBoolean;

public class Arm extends SubsystemBase {
    private DBugSparkMax _leader;
    private DBugSparkMax _follower;
    private SparkMaxLimitSwitch _forwardLimit;
    private SparkMaxLimitSwitch _reverseLimit;
    private LatchedBoolean _forwardState;
    private LatchedBoolean _reverseState;
    private RelativeEncoder _encoder;
    private SparkMaxPIDController _PIDController;
    private ArmFeedforward _feedforward;

    private double _lastGoal;

    private static DBugSparkMax createSparkMax(int id) {
        DBugSparkMax sparkMax = DBugSparkMax.create(
            id,
            ArmConstants.gains,
            ArmConstants.motorToArmConversionFactor,
            ArmConstants.motorToArmConversionFactor / 60,
            ArmConstants.startingAngle);

        sparkMax.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen).enableLimitSwitch(true);
        sparkMax.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyClosed).enableLimitSwitch(true);
        return sparkMax;
    }

    public Arm() {
        _leader = createSparkMax(ArmConstants.leaderCANID);
        _follower = createSparkMax(ArmConstants.followerCANID);

        _leader.setInverted(ArmConstants.motorInverted);
        _follower.follow(_leader, true);

        _forwardState = new LatchedBoolean();
        _reverseState = new LatchedBoolean();

        _feedforward = new ArmFeedforward(0, ArmConstants.gravityFF, ArmConstants.velocityFF);

        _lastGoal = ArmConstants.startingAngle;

        // initSDB();
    }

    private void updatePIDFromSDB() {
        _PIDController.setP(SmartDashboard.getNumber("P Gain", ArmConstants.kP), ArmConstants.kPIDSlot);

        double kMaxOutput = SmartDashboard.getNumber("Max Output", ArmConstants.kMaxOutput);
        _PIDController.setOutputRange(-kMaxOutput, kMaxOutput, ArmConstants.kPIDSlot);
    }

    private void updateFeedForwardFromSDB() {
        _feedforward = new ArmFeedforward(0,
                SmartDashboard.getNumber("Gravity Gain", ArmConstants.gravityFF),
                SmartDashboard.getNumber("Velocity Gain", ArmConstants.velocityFF));
    }

    public boolean isLastGoalIntake() {
        return (_lastGoal == ArmConstants.intakeAngle);
    }

    public Command getActiveGoalCommand(double angle) {
        _lastGoal = angle;

        TrapezoidProfile _profile = new TrapezoidProfile(
                ArmConstants.trapezoidConstraints,
                new TrapezoidProfile.State(angle, 0),
                new TrapezoidProfile.State(_encoder.getPosition(), _encoder.getVelocity()));

        return new TrapezoidProfileCommand(_profile, this::useState, this);
    }

    private Command getActiveGoalFromSDB() {
        return getActiveGoalCommand(SmartDashboard.getNumber("Arm Goal", ArmConstants.startingAngle));

    }

    public void useState(TrapezoidProfile.State state) {

        double feedforward = _feedforward.calculate(Math.toRadians(state.position), state.velocity);

        _PIDController.setReference(state.position, ControlType.kPosition, ArmConstants.kPIDSlot, feedforward,
                ArbFFUnits.kPercentOut);

        // updateSDB(state, feedforward);
    }

    public void disabledInit() {
        _leader.set(0);
    }

    @SuppressWarnings("unused")
    private void initSDB() {
        SmartDashboard.setDefaultNumber("P Gain", ArmConstants.kP);
        SmartDashboard.setDefaultNumber("Max Output", ArmConstants.kMaxOutput);
        SmartDashboard.setDefaultNumber("Arm Goal", ArmConstants.startingAngle);
        SmartDashboard.setDefaultNumber("Gravity Gain", ArmConstants.gravityFF);
        SmartDashboard.setDefaultNumber("Velocity Gain", ArmConstants.velocityFF);

        SmartDashboard.putData("Update PID", new InstantCommand(() -> updatePIDFromSDB()));
        SmartDashboard.putData("Set Feed Forward", new InstantCommand(() -> updateFeedForwardFromSDB()));
        SmartDashboard.putData("Set Arm Goal", new InstantCommand(() -> getActiveGoalFromSDB().schedule()));
    }

    @SuppressWarnings("unused")
    private void updateSDB(TrapezoidProfile.State state, double feedforward) {
        SmartDashboard.putBoolean("Forward Limit pressed", _forwardLimit.isPressed());
        SmartDashboard.putBoolean("Reverse Limit pressed", _reverseLimit.isPressed());

        SmartDashboard.putNumber("Arm Position", _encoder.getPosition());
        SmartDashboard.putNumber("Arm Velocity", _encoder.getVelocity());

        SmartDashboard.putNumber("Arm State Position", state.position);
        SmartDashboard.putNumber("Arm State Velocity", state.velocity);

        SmartDashboard.putNumber("Arm Feed Forward", feedforward);
    }

    @SuppressWarnings("unused")
    private void updateSDB() {
        SmartDashboard.putBoolean("Forward Limit pressed", _forwardLimit.isPressed());
        SmartDashboard.putBoolean("Reverse Limit pressed", _reverseLimit.isPressed());

        SmartDashboard.putNumber("Arm Position", _encoder.getPosition());
        SmartDashboard.putNumber("Arm Velocity", _encoder.getVelocity());
    }

    @Override
    public void periodic() {
        if (_forwardState.update(_forwardLimit.isPressed())) {
            _encoder.setPosition(ArmConstants.intakeAngle);

        } else if (_reverseState.update(_reverseLimit.isPressed())) {
            _encoder.setPosition(ArmConstants.shootAngle);
        }
        // updateSDB();
    }
}