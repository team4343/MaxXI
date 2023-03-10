package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import io.github.oblarg.oblog.Loggable;

public class ArmSubsystem extends SubsystemBase implements Loggable {
    public enum State {
        Pickup, Rest, PlacingA, PlacingB, PlacingC, PlacingD, PlacingE, Transit // Possible System States
    }

    private static final double elbowPosToDegrees = 599.9;
    private static final double shoulderPosToDegrees = 342.8;
    private static final double stateTolerance = 2;
    private static int loops = 0;

    // System motor controllers
    private static final CANSparkMax m_shoulder = new CANSparkMax(ArmConstants.SHOULDER_ID, MotorType.kBrushless);
    private static final CANSparkMax m_shoulderFollower = new CANSparkMax(ArmConstants.SHOULDER_FOLLOWER_ID, MotorType.kBrushless);
    private static final CANSparkMax m_elbow = new CANSparkMax(ArmConstants.ELBOW_ID, MotorType.kBrushless);

    // System State - init in rest state
    private static State m_state_desired = State.Rest;
    private static State m_state_actual = State.Rest;

    private static final NetworkTableInstance handle = NetworkTableInstance.getDefault();

    public void printState() {
        System.out.print("Desired: ");
        System.out.print(this.m_state_desired);
        System.out.print("\t\t Actual: ");
        System.out.print(this.m_state_actual);
    }

    // Class to hold position Data for each system state
    private static class POS {
        public double elbow, shoulder = 0;
        public POS(double shoulder, double elbow) {
            this.elbow = elbow;
            this.shoulder = shoulder;
        }
    }

    private static class PID {
        public int slot = 0;
        public double P, I, D, FF, DF = 0;
        public PID(double P, double I, double D, int slot) {
            this.P = P;
            this.I = I;
            this.D = D;
            this.slot = slot;
        }
    }

    // Position Constants
    // Horizontal = 40, Down = 0, Up = 80
    private final POS REST      = new POS(0, 0);
    private final POS PICKUP    = new POS(0, 14);
//    private final POS PLACING_A = new POS(9.5, 10); For the top row this works
    private final POS PLACING_A = new POS(7, 14);
    private final POS PLACING_B = new POS(5, 6);
    private final POS PLACING_C = new POS(0, 0);
    private final POS PLACING_D = new POS(0, 0);
    private final POS PLACING_E = new POS(0, 0);

    // PID SLOT IDS
    // P. Proportional output to the error of the system
    // I. Sum of error over time. This increases output to counteract steady state error
    // D. Rate of change of error. This decreases output to counteract oscillation
    private static final PID SHOULDER_DEFAULT  = new PID(0.11, 0, 0, 0);
    private static final PID SHOULDER_STEADY   = new PID(0.15, 0, 0, 1);
    private static final PID ELBOW_DEFAULT     = new PID(0.075, 0, 0, 0);
    private static final PID ELBOW_STEADY      = new PID(0.10, 0, 0, 1);

    private final Double SHOULDER_RAMP_RATE = 0.5;
    private final Double ELBOW_RAMP_RATE = 0.5;

    private final float SHOULDER_MAX_POS = 9.5f;
    private final float SHOULDER_MIN_POS = 0;
    private final float ELBOW_MAX_POS = 20;
    private final float ELBOW_MIN_POS = 0;


    public ArmSubsystem() {
        m_elbow.setInverted(true);

        // Follow the main shoulder encoder, inverted.
        m_shoulderFollower.follow(m_shoulder, true);

        var tab = Shuffleboard.getTab("ArmSubsystem");
        tab.addString("Desired State", () -> m_state_desired.name());
        tab.addString("Actual State", () -> m_state_actual.name());


        // ****** INITIALIZATION
        // Set the idle mode to brake
        m_shoulder.setIdleMode(CANSparkMax.IdleMode.kBrake);
        m_shoulderFollower.setIdleMode(CANSparkMax.IdleMode.kBrake);
        m_elbow.setIdleMode(CANSparkMax.IdleMode.kBrake);

        // Set the peak and nominal output
        m_shoulder.setSmartCurrentLimit(60);
        m_elbow.setSmartCurrentLimit(60);

        // Set the ramp rate of 0.25 seconds to full power
        m_shoulder.setClosedLoopRampRate(SHOULDER_RAMP_RATE);
        m_elbow.setClosedLoopRampRate(ELBOW_RAMP_RATE);

        m_shoulder.setControlFramePeriodMs(40);
        m_elbow.setControlFramePeriodMs(40);

        m_shoulder.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 500);
        m_shoulder.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 500);
        m_shoulderFollower.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 100);
        m_shoulderFollower.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 500);
        m_shoulderFollower.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 500);

        // Set default PID values for shoulder on all slots
        for (var config : new PID[]{SHOULDER_DEFAULT, SHOULDER_STEADY}) {
            m_shoulder.getPIDController().setP(config.P, config.slot);
            m_shoulder.getPIDController().setI(config.I, config.slot);
            m_shoulder.getPIDController().setD(config.D, config.slot);
            m_shoulder.getPIDController().setFF(config.FF, config.slot);
            m_shoulder.getPIDController().setDFilter(config.DF, config.slot);
        }

        // Set default PID values elbow
        for (var config : new PID[]{ELBOW_DEFAULT, ELBOW_STEADY}) {
            m_elbow.getPIDController().setP(config.P, config.slot);
            m_elbow.getPIDController().setI(config.I, config.slot);
            m_elbow.getPIDController().setD(config.D, config.slot);
            m_elbow.getPIDController().setFF(config.FF, config.slot);
            m_elbow.getPIDController().setDFilter(config.DF, config.slot);
        }

        // Set the soft limits
        m_shoulder.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
        m_shoulder.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
        m_elbow.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
        m_elbow.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
        m_shoulder.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, SHOULDER_MAX_POS);
        m_shoulder.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, SHOULDER_MIN_POS);
        m_elbow.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, ELBOW_MAX_POS);
        m_elbow.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, ELBOW_MIN_POS);
    }

    public void setState(State state) {
        // Don't do anything already in correct state
        if (state == m_state_actual)
            return;

        switch (m_state_actual) {
            case Pickup:
            case Transit:
                if (state == State.Rest)
                    m_state_desired = state;
                break;
            case Rest:
            case PlacingA:
            case PlacingB:
            case PlacingC:
            case PlacingD:
            case PlacingE:
                m_state_desired = state;
                break;
        }
    }

    public ArmSubsystem.State deserializeStateFromString(String stateName) {
        System.out.println("Deserializing state for " + stateName);

        switch (stateName) {
            case "Rest": return State.Rest;
            case "Pickup": return State.Pickup;
            case "PlacingA": return State.PlacingA;
            case "PlacingB": return State.PlacingB;
            case "PlacingC": return State.PlacingC;
            case "PlacingD": return State.PlacingD;
            case "PlacingE": return State.PlacingE;
        }

        return ArmSubsystem.State.Rest;
    }

    public void periodic() {
        matchActualState();
        POS position;
        PID shoulder_pid;
        PID elbow_pid;

        // If the state is not the same as the desired state, then run the periodic for that state.
        if (m_state_actual != m_state_desired) {
            shoulder_pid = SHOULDER_DEFAULT;
            elbow_pid = ELBOW_DEFAULT;
        } else {
            shoulder_pid = SHOULDER_STEADY;
            elbow_pid = ELBOW_STEADY;
        }

        switch (m_state_desired) {
            case Pickup: position=PICKUP; break;
            case PlacingA: position=PLACING_A; break;
            case PlacingB: position=PLACING_B; break;
            case PlacingC: position=PLACING_C; break;
            case PlacingD: position=PLACING_D; break;
            case PlacingE: position=PLACING_E; break;
            default: position=REST;
        }

        m_elbow.getPIDController().setReference(position.elbow,
                CANSparkMax.ControlType.kPosition,
                elbow_pid.slot);

        m_shoulder.getPIDController().setReference(position.shoulder,
                CANSparkMax.ControlType.kPosition,
                shoulder_pid.slot);

        // Update the dashboard
        SmartDashboard.putNumber("Shoulder Position", m_shoulder.getEncoder().getPosition());
        SmartDashboard.putNumber("Elbow Position", m_elbow.getEncoder().getPosition());

        handle.getEntry("/ArmSubsystem/DesiredState").setString(String.valueOf(m_state_desired));
        handle.getEntry("/ArmSubsystem/ActualState").setString(String.valueOf(m_state_actual));
    }

    private static boolean inRange(double value, double target, double tolerance) {
        return value >= target - tolerance && value <= target + tolerance;
    }

    private void matchActualState() {
        // Record the actual_state to the state that the arm is in based on encoder values
        double shoulder_pos = m_shoulder.getEncoder().getPosition();
        double elbow_pos = m_elbow.getEncoder().getPosition();

        if (inRange(shoulder_pos, REST.shoulder, stateTolerance) && inRange(elbow_pos, REST.elbow, stateTolerance))
            m_state_actual = State.Rest;
        else if (inRange(shoulder_pos, PICKUP.shoulder, stateTolerance) && inRange(elbow_pos, PICKUP.elbow, stateTolerance))
            m_state_actual = State.Pickup;
        else if (inRange(shoulder_pos, PLACING_A.shoulder, stateTolerance) && inRange(elbow_pos, PLACING_A.elbow, stateTolerance))
            m_state_actual = State.PlacingA;
        else if (inRange(shoulder_pos, PLACING_B.shoulder, stateTolerance) && inRange(elbow_pos, PLACING_B.elbow, stateTolerance))
            m_state_actual = State.PlacingB;
        else if (inRange(shoulder_pos, PLACING_C.shoulder, stateTolerance) && inRange(elbow_pos, PLACING_C.elbow, stateTolerance))
            m_state_actual = State.PlacingC;
        else if (inRange(shoulder_pos, PLACING_D.shoulder, stateTolerance) && inRange(elbow_pos, PLACING_D.elbow, stateTolerance))
            m_state_actual = State.PlacingD;
        else if (inRange(shoulder_pos, PLACING_E.shoulder, stateTolerance) && inRange(elbow_pos, PLACING_E.elbow, stateTolerance))
            m_state_actual = State.PlacingE;

        if (m_state_actual != m_state_desired)
            m_state_actual = State.Transit;
    }

    private double  convert(double desired) {
        return desired * shoulderPosToDegrees;
    }

}
