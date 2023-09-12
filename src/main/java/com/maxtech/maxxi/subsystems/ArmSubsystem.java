package com.maxtech.maxxi.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static com.maxtech.maxxi.constants.MotorConstants.*;

public class ArmSubsystem extends SubsystemBase {
    public enum State {
        Rest, PickupGround, PickupStation, PlacingMiddle, PLacingUpper, PlaceholderA, PlaceholderB, Transit // Possible System States
    }

    // Tolerance of deciding if the arm is in a state (encoder count +/- tolerance)
    private static final double stateTolerance = 1;
    private static double startTime = 0.0;

    // System motor controllers
    private static final CANSparkMax shoulder = new CANSparkMax(SHOULDER_ID, MotorType.kBrushless);
    private static final CANSparkMax shoulderFollower = new CANSparkMax(SHOULDER_FOLLOWER_ID, MotorType.kBrushless);
    private static final CANSparkMax elbow = new CANSparkMax(ELBOW_ID, MotorType.kBrushless);
    private static final CANSparkMax wrist = new CANSparkMax(WRIST_ID, MotorType.kBrushless);

    // System State - init in rest state
    private static State state_desired = State.Rest;
    private static State state_desired_previous = State.Rest;
    private static State state_actual = State.Rest;
    private static State state_previous_timer = State.Rest;
    private static State state_previous = State.Rest;

    private static final NetworkTableInstance nt_handle = NetworkTableInstance.getDefault();

    // Class to hold position Data for each system state
    private static class POS {
        public double elbow, shoulder, wrist;
        public POS(double shoulder, double elbow, double wrist) {
            this.elbow = elbow;
            this.shoulder = shoulder;
            this.wrist = wrist;
        }
    }

    private static class PID {
        public int slot;
        public double P, I, D, FF, DF = 0;
        public PID(double P, double I, double D, int slot) {
            this.P = P;
            this.I = I;
            this.D = D;
            this.slot = slot;
        }
    }

    // Position Constants
    private final POS REST              = new POS(1, -2, -10);
    private final POS PICKUP_GROUND     = new POS(0, 15, 13);
    private final POS PLACING_MIDDLE    = new POS(1.5, 7, 20);
    private final POS PLACING_UPPER     = new POS(7.2, 15.5, 20);
    private final POS PICKUP_STATION    = new POS(6.9, 15, 20);
    private final POS INIT              = new POS(0, 0, 0);
    private final POS PLACEHOLDER_B     = new POS(0, 0, 0);

    // PID Control
    // P. Proportional output to the error of the system
    // I. Sum of error over time. This increases output to counteract steady state error
    // D. Rate of change of error. This decreases output to counteract oscillation
    // Slot. So we can rapidly switch between PID configurations.
    private static final PID SHOULDER_DEFAULT  = new PID(0.13, 0.000025, 0, 0);
    private static final PID SHOULDER_STEADY   = new PID(0.2, 0.0007, 0, 1);
    private static final PID ELBOW_DEFAULT     = new PID(0.045, 0.00001, 0, 0);
    private static final PID ELBOW_STEADY      = new PID(0.055, 0.00017, 0, 1);
    private static final PID ELBOW_PICKUP      = new PID(0.035, 0.00002, 0.00013, 2);
    private static final PID WRIST_DEFAULT     = new PID(0.02, 0.00000, 0, 0);

    private final Double retractDelay = 0.6;
    private final Double SHOULDER_RAMP_RATE = 0.4;
    private final Double ELBOW_RAMP_RATE = 0.3; 
    private final Double WRIST_RAMP_RATE = 0.5;

    private final float SHOULDER_MAX_POS = 9.0f;
    private final float SHOULDER_MIN_POS = -3;
    private final float ELBOW_MAX_POS = 20;
    private final float ELBOW_MIN_POS = -3;
    private final float WRIST_MAX_POS = 30;
    private final float WRIST_MIN_POS = -10;


    public ArmSubsystem() {
        elbow.setInverted(true);
        wrist.setInverted(false);

        // Follow the main shoulder encoder, inverted.
        shoulderFollower.follow(shoulder, true);

        // Set the idle mode to brake
        shoulder.setIdleMode(CANSparkMax.IdleMode.kBrake);
        shoulderFollower.setIdleMode(CANSparkMax.IdleMode.kBrake);
        elbow.setIdleMode(CANSparkMax.IdleMode.kBrake);
        wrist.setIdleMode(CANSparkMax.IdleMode.kBrake);

        // Set the peak and nominal output
        shoulder.setSmartCurrentLimit(80);
        elbow.setSmartCurrentLimit(80);
        wrist.setSmartCurrentLimit(60);

        // Set the ramp rate of X seconds to full power
        shoulder.setClosedLoopRampRate(SHOULDER_RAMP_RATE);
        elbow.setClosedLoopRampRate(ELBOW_RAMP_RATE);
        wrist.setClosedLoopRampRate(WRIST_RAMP_RATE);

        shoulder.setControlFramePeriodMs(20);
        elbow.setControlFramePeriodMs(20);
        wrist.setControlFramePeriodMs(40);


        // Set default PID values for shoulder on all slots
        for (var config : new PID[]{SHOULDER_DEFAULT, SHOULDER_STEADY}) {
            shoulder.getPIDController().setP(config.P, config.slot);
            shoulder.getPIDController().setI(config.I, config.slot);
            shoulder.getPIDController().setD(config.D, config.slot);
            shoulder.getPIDController().setFF(config.FF, config.slot);
            shoulder.getPIDController().setDFilter(config.DF, config.slot);
        }

        // Set default PID values elbow
        for (var config : new PID[]{ELBOW_DEFAULT, ELBOW_STEADY, ELBOW_PICKUP}) {
            elbow.getPIDController().setP(config.P, config.slot);
            elbow.getPIDController().setI(config.I, config.slot);
            elbow.getPIDController().setD(config.D, config.slot);
            elbow.getPIDController().setFF(config.FF, config.slot);
            elbow.getPIDController().setDFilter(config.DF, config.slot);
        }

        // Set default PID values wrist
        for (var config : new PID[]{WRIST_DEFAULT}) {
            wrist.getPIDController().setP(config.P, config.slot);
            wrist.getPIDController().setI(config.I, config.slot);
            wrist.getPIDController().setD(config.D, config.slot);
            wrist.getPIDController().setFF(config.FF, config.slot);
            wrist.getPIDController().setDFilter(config.DF, config.slot);
        }

        // Set the soft limits
        shoulder.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
        shoulder.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
        elbow.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
        elbow.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
        wrist.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
        wrist.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
        shoulder.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, SHOULDER_MAX_POS);
        shoulder.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, SHOULDER_MIN_POS);
        elbow.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, ELBOW_MAX_POS);
        elbow.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, ELBOW_MIN_POS);
        wrist.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, WRIST_MAX_POS);
        wrist.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, WRIST_MIN_POS);
    }

    public void setState(State state) {
        // Don't do anything already in correct state
        if (state == state_actual)
            return;

        switch (state_actual) {
            case PickupGround:
            case Transit:
                if (state == State.Rest)
                    state_desired = state;
                break;
            case Rest:
            case PlacingMiddle:
            case PLacingUpper:
            case PickupStation:
            case PlaceholderA:
            case PlaceholderB:
                state_desired = state;
                break;
        }
    }

    public void periodic() {
        matchActualState();
        POS position;
        int shoulder_pid_slot;
        int elbow_pid_slot;
        int wrist_pid_slot;

        if (state_desired == state_actual && state_actual == state_previous)
            return;

        if (state_desired != state_previous_timer)
            startTime = Timer.getFPGATimestamp();

        // If the state is not the same as the desired state, then run the periodic for that state.
        if (!(state_previous == state_actual || state_previous == state_desired))  {
            shoulder.getPIDController().setIAccum(0);
            elbow.getPIDController().setIAccum(0);
            wrist.getPIDController().setIAccum(0);
        }

        if (state_actual != state_desired) {
            shoulder_pid_slot = SHOULDER_DEFAULT.slot;
            elbow_pid_slot = ELBOW_DEFAULT.slot;
            wrist_pid_slot = WRIST_DEFAULT.slot;
            SmartDashboard.putString("Elbow State", "Default");
            SmartDashboard.putString("Arm State", "Default");
            if (state_desired == State.PickupGround) {
                elbow_pid_slot = ELBOW_PICKUP.slot;
                SmartDashboard.putString("Elbow State", "Ground");
            }
        } else {
            shoulder_pid_slot = SHOULDER_STEADY.slot;
            elbow_pid_slot = ELBOW_STEADY.slot;
            wrist_pid_slot = WRIST_DEFAULT.slot;
            SmartDashboard.putString("Elbow State", "Steady");
            SmartDashboard.putString("Arm State", "Steady");
        }

        switch (state_desired) {
            case PickupGround: position= PICKUP_GROUND; break;
            case PlacingMiddle: position= PLACING_MIDDLE; break;
            case PLacingUpper: position= PLACING_UPPER; break;
            case PickupStation: position= PICKUP_STATION; break;
//            case PlaceholderA: position= INIT; break;
//            case PlaceholderB: position= PLACEHOLDER_B; break;
            default: position=REST;
        }

        if (state_desired == State.PickupGround)
            elbow.setIdleMode(CANSparkMax.IdleMode.kCoast);
        else
            elbow.setIdleMode(CANSparkMax.IdleMode.kBrake);

        state_previous = state_actual;
        state_previous_timer = state_desired;

        // Actual Control
        elbow.getPIDController().setReference(position.elbow, CANSparkMax.ControlType.kPosition, elbow_pid_slot);
        if (startTime + retractDelay < Timer.getFPGATimestamp() || state_desired != State.Rest)
            shoulder.getPIDController().setReference(position.shoulder, CANSparkMax.ControlType.kPosition, shoulder_pid_slot);
        wrist.getPIDController().setReference(position.wrist, CANSparkMax.ControlType.kPosition, wrist_pid_slot);

        SmartDashboard.putNumber("ShoulderPosition", shoulder.getEncoder().getPosition());
        SmartDashboard.putNumber("ElbowPosition", elbow.getEncoder().getPosition());
        SmartDashboard.putNumber("WristPosition", wrist.getEncoder().getPosition());
    }

    private static boolean inRange(double value, double target) {
        return value >= target - stateTolerance && value <= target + stateTolerance;
    }

    private void matchActualState() {
        // Record the actual_state to the state that the arm is in based on encoder values
        double shoulder_pos = shoulder.getEncoder().getPosition();
        double elbow_pos = elbow.getEncoder().getPosition();

        if (inRange(shoulder_pos, REST.shoulder) && inRange(elbow_pos, REST.elbow))
            state_actual = State.Rest;
        else if (inRange(shoulder_pos, PICKUP_GROUND.shoulder) && inRange(elbow_pos, PICKUP_GROUND.elbow))
            state_actual = State.PickupGround;
        else if (inRange(shoulder_pos, PLACING_MIDDLE.shoulder) && inRange(elbow_pos, PLACING_MIDDLE.elbow))
            state_actual = State.PlacingMiddle;
        else if (inRange(shoulder_pos, PLACING_UPPER.shoulder) && inRange(elbow_pos, PLACING_UPPER.elbow))
            state_actual = State.PLacingUpper;
        else if (inRange(shoulder_pos, PICKUP_STATION.shoulder) && inRange(elbow_pos, PICKUP_STATION.elbow))
            state_actual = State.PickupStation;

        SmartDashboard.putString("Arm State Actual", state_actual.toString());
        SmartDashboard.putString("Arm State Desired", state_desired.toString());

        if (state_actual != state_desired)
            state_actual = State.Transit;
    }

}
