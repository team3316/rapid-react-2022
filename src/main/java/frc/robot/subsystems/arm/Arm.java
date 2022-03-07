package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.SparkMaxLimitSwitch;
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
import frc.robot.motors.PIDFGains;
import frc.robot.utils.LatchedBoolean;
import frc.robot.utils.Within;

public class Arm extends SubsystemBase {
    private DBugSparkMax _leader;
    private DBugSparkMax _follower;
    private SparkMaxLimitSwitch _forwardLimit;
    private SparkMaxLimitSwitch _reverseLimit;
    private LatchedBoolean _forwardState;
    private ArmFeedforward _feedforward;

    private double _lastGoal;

    public Arm() {
        _leader = DBugSparkMax.create(ArmConstants.leaderCANID,
                ArmConstants.gains,
                ArmConstants.motorToArmConversionFactor,
                ArmConstants.motorToArmConversionFactor / 60,
                ArmConstants.startingAngle);
        _follower = DBugSparkMax.create(ArmConstants.followerCANID,
                ArmConstants.gains,
                ArmConstants.motorToArmConversionFactor,
                ArmConstants.motorToArmConversionFactor / 60,
                ArmConstants.startingAngle);

        _leader.setInverted(ArmConstants.motorInverted);
        _follower.follow(_leader, true);

        enableLimitSwitch();

        _forwardState = new LatchedBoolean();

        _feedforward = new ArmFeedforward(0, ArmConstants.gravityFF, ArmConstants.velocityFF);

        _lastGoal = getArmInitPosition();
        _leader.setPosition(getArmInitPosition());

        // initSDB();
    }

    private void enableLimitSwitch() {
        _forwardLimit = _leader.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
        _forwardLimit.enableLimitSwitch(true);
        _reverseLimit = _leader.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
        _reverseLimit.enableLimitSwitch(true);
    }

    private double getArmInitPosition() {
        if (_forwardState.update(_forwardLimit.isPressed())) {
            return ArmConstants.intakeAngle;
        }
        if (_forwardState.update(_reverseLimit.isPressed())) {
            return ArmConstants.shootAngle;
        }
        return ArmConstants.startingAngle;
    }

    private void updatePIDFromSDB() {
        double kP = SmartDashboard.getNumber("P Gain", ArmConstants.kP);
        double kMaxOutput = SmartDashboard.getNumber("Max Output", ArmConstants.kMaxOutput);
        _leader.setupPIDF(new PIDFGains(kP, 0, 0, 0, kMaxOutput));
    }

    private void updateFeedForwardFromSDB() {
        _feedforward = new ArmFeedforward(0,
                SmartDashboard.getNumber("Gravity Gain", ArmConstants.gravityFF),
                SmartDashboard.getNumber("Velocity Gain", ArmConstants.velocityFF));
    }

    public boolean isLastGoalIntake() {
        return (_lastGoal == ArmConstants.intakeAngle);
    }

    public boolean atGoal(){
        return Within.range(_leader.getPosition(), _lastGoal, 3.5);
    }

    public Command getActiveGoalCommand(double angle) {
        _lastGoal = angle;

        TrapezoidProfile _profile = new TrapezoidProfile(
                ArmConstants.trapezoidConstraints,
                new TrapezoidProfile.State(angle, 0),
                new TrapezoidProfile.State(_leader.getPosition(), _leader.getVelocity()));

        return new TrapezoidProfileCommand(_profile, this::useState, this);
    }

    private Command getActiveGoalFromSDB() {
        return getActiveGoalCommand(SmartDashboard.getNumber("Arm Goal", ArmConstants.startingAngle));

    }

    public void useState(TrapezoidProfile.State state) {

        double feedforward = _feedforward.calculate(Math.toRadians(state.position), state.velocity);

        _leader.setReference(state.position, ControlType.kPosition, 0, feedforward, ArbFFUnits.kPercentOut);

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

        SmartDashboard.putNumber("Arm Position", _leader.getPosition());
        SmartDashboard.putNumber("Arm Velocity", _leader.getVelocity());

        SmartDashboard.putNumber("Arm State Position", state.position);
        SmartDashboard.putNumber("Arm State Velocity", state.velocity);

        SmartDashboard.putNumber("Arm Feed Forward", feedforward);
    }

    // @SuppressWarnings("unused")
    private void updateSDB() {
        SmartDashboard.putBoolean("Forward Limit pressed", _forwardLimit.isPressed());
        SmartDashboard.putBoolean("Reverse Limit pressed", _reverseLimit.isPressed());

        SmartDashboard.putNumber("Arm Position", _leader.getPosition());
        // SmartDashboard.putNumber("Arm Velocity", _leader.getVelocity());
    }

    @Override
    public void periodic() {
        if (_forwardState.update(_forwardLimit.isPressed())) {
            _leader.setPosition(ArmConstants.intakeAngle - ArmConstants.overshootDelta);
        }

        updateSDB();
    }

    public void setArmEncoder(double angle) {
        _leader.setPosition(angle);
    }
}