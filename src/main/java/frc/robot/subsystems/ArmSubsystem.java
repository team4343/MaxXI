package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.MotorConstants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {
    public enum State {
        Pickup, Rest, PlacingA, PlacingB, PlacingC, PlacingD, PlacingE, Transit // Possible System States
    }

    // Tolerance of deciding if the arm is in a state (encoder count +/- tolerance)
    private static final double stateTolerance = 1;

    // System motor controllers
    private static final CANSparkMax m_shoulder = new CANSparkMax(ArmConstants.SHOULDER_ID, MotorType.kBrushless);
    private static final CANSparkMax m_shoulderFollower = new CANSparkMax(ArmConstants.SHOULDER_FOLLOWER_ID, MotorType.kBrushless);
    private static final CANSparkMax m_elbow = new CANSparkMax(ArmConstants.ELBOW_ID, MotorType.kBrushless);
    private static final CANSparkMax m_wrist = new CANSparkMax(ArmConstants.WRIST_ID, MotorType.kBrushless);

    // System State - init in rest state
    private static State m_state_desired = State.Rest;
    private static State m_state_actual = State.Rest;

    private static final NetworkTableInstance nt_handle = NetworkTableInstance.getDefault();

    // Class to hold position Data for each system state
    private static class POS {
        public double elbow, shoulder, wrist = 0;
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
        public PID(double P, double I, double D, double FF, int slot) {
            this.P = P;
            this.I = I;
            this.D = D;
            this.FF = FF;
            this.slot = slot;
        }
    }

    // Position Constants
    private final POS REST      = new POS(0, 0, 10);
    private final POS PICKUP    = new POS(0, 13, 60);
    private final POS PLACING_A = new POS(2, 5, 50);
    private final POS PLACING_B = new POS(7.5, 14.5, 60);
    private final POS PLACING_C = new POS(0, 0, 0);
    private final POS PLACING_D = new POS(0, 0, 0);
    private final POS PLACING_E = new POS(0, 0, 0);

    // PID Control
    // P. Proportional output to the error of the system
    // I. Sum of error over time. This increases output to counteract steady state error
    // D. Rate of change of error. This decreases output to counteract oscillation
    // Slot. So we can rapidly switch between PID configurations.
    private static final PID SHOULDER_DEFAULT  = new PID(0.1, 0.00012, 0, 0);
    private static final PID SHOULDER_STEADY   = new PID(0.12, 0.00017, 0, 1);
    private static final PID ELBOW_DEFAULT     = new PID(0.025, 0.000014, 0, 0);
    private static final PID ELBOW_STEADY      = new PID(0.03, 0.00002
    , 0, 1);
    private static final PID ELBOW_PICKUP      = new PID(0.04, 0, 0, 2);
    private static final PID WRIST_DEFAULT     = new PID(0.02, 0.00000, 0, 0);

    private final Double SHOULDER_RAMP_RATE = 0.5;
    private final Double ELBOW_RAMP_RATE = 0.5; 
    private final Double WRIST_RAMP_RATE = 0.5;

    private final float SHOULDER_MAX_POS = 9.5f;
    private final float SHOULDER_MIN_POS = 0;
    private final float ELBOW_MAX_POS = 20;
    private final float ELBOW_MIN_POS = 0;
    private final float WRIST_MAX_POS = 60;
    private final float WRIST_MIN_POS = -10;


    public ArmSubsystem() {
        m_elbow.setInverted(true);

        // Follow the main shoulder encoder, inverted.
        m_shoulderFollower.follow(m_shoulder, true);


        // Set the idle mode to brake
        m_shoulder.setIdleMode(CANSparkMax.IdleMode.kBrake);
        m_shoulderFollower.setIdleMode(CANSparkMax.IdleMode.kBrake);
        m_elbow.setIdleMode(CANSparkMax.IdleMode.kBrake);
        m_wrist.setIdleMode(CANSparkMax.IdleMode.kBrake);

        // Set the peak and nominal output
        m_shoulder.setSmartCurrentLimit(80);
        m_elbow.setSmartCurrentLimit(60);
        m_wrist.setSmartCurrentLimit(60);

        // Set the ramp rate of 0.25 seconds to full power
        m_shoulder.setClosedLoopRampRate(SHOULDER_RAMP_RATE);
        m_elbow.setClosedLoopRampRate(ELBOW_RAMP_RATE);
        m_wrist.setClosedLoopRampRate(WRIST_RAMP_RATE);

        m_shoulder.setControlFramePeriodMs(20);
        m_elbow.setControlFramePeriodMs(40);
        m_wrist.setControlFramePeriodMs(40);


        // Set default PID values for shoulder on all slots
        for (var config : new PID[]{SHOULDER_DEFAULT, SHOULDER_STEADY}) {
            m_shoulder.getPIDController().setP(config.P, config.slot);
            m_shoulder.getPIDController().setI(config.I, config.slot);
            m_shoulder.getPIDController().setD(config.D, config.slot);
            m_shoulder.getPIDController().setFF(config.FF, config.slot);
            m_shoulder.getPIDController().setDFilter(config.DF, config.slot);
        }

        // Set default PID values elbow
        for (var config : new PID[]{ELBOW_DEFAULT, ELBOW_STEADY, ELBOW_PICKUP}) {
            m_elbow.getPIDController().setP(config.P, config.slot);
            m_elbow.getPIDController().setI(config.I, config.slot);
            m_elbow.getPIDController().setD(config.D, config.slot);
            m_elbow.getPIDController().setFF(config.FF, config.slot);
            m_elbow.getPIDController().setDFilter(config.DF, config.slot);
        }

        // Set default PID values wrist
        for (var config : new PID[]{WRIST_DEFAULT}) {
            m_wrist.getPIDController().setP(config.P, config.slot);
            m_wrist.getPIDController().setI(config.I, config.slot);
            m_wrist.getPIDController().setD(config.D, config.slot);
            m_wrist.getPIDController().setFF(config.FF, config.slot);
            m_wrist.getPIDController().setDFilter(config.DF, config.slot);
        }

        // Set the soft limits
        m_shoulder.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
        m_shoulder.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
        m_elbow.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
        m_elbow.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
        m_wrist.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
        m_wrist.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
        m_shoulder.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, SHOULDER_MAX_POS);
        m_shoulder.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, SHOULDER_MIN_POS);
        m_elbow.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, ELBOW_MAX_POS);
        m_elbow.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, ELBOW_MIN_POS);
        m_wrist.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, WRIST_MAX_POS);
        m_wrist.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, WRIST_MIN_POS);
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
        int shoulder_pid_slot;
        int elbow_pid_slot;
        int wrist_pid_slot;

        // If the state is not the same as the desired state, then run the periodic for that state.
        if (m_state_actual != m_state_desired) {
            shoulder_pid_slot = SHOULDER_DEFAULT.slot;
            elbow_pid_slot = ELBOW_DEFAULT.slot;
            wrist_pid_slot = WRIST_DEFAULT.slot;
            if (m_state_desired == State.Pickup)
                elbow_pid_slot = ELBOW_PICKUP.slot;
        } else {
            shoulder_pid_slot = SHOULDER_STEADY.slot;
            elbow_pid_slot = ELBOW_STEADY.slot;
            wrist_pid_slot = WRIST_DEFAULT.slot;
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

        m_elbow.getPIDController().setReference(position.elbow, CANSparkMax.ControlType.kPosition, elbow_pid_slot);
        m_shoulder.getPIDController().setReference(position.shoulder, CANSparkMax.ControlType.kPosition, shoulder_pid_slot);
        m_wrist.getPIDController().setReference(position.wrist, CANSparkMax.ControlType.kPosition, wrist_pid_slot);

        nt_handle.getEntry("/ArmSubsystem/DesiredState").setString(String.valueOf(m_state_desired));
        nt_handle.getEntry("/ArmSubsystem/ActualState").setString(String.valueOf(m_state_actual));
        nt_handle.getEntry("/ArmSubsystem/ShoulderPosition").setDouble(m_shoulder.getEncoder().getPosition());
        nt_handle.getEntry("/ArmSubsystem/ElbowPosition").setDouble(m_elbow.getEncoder().getPosition());
        nt_handle.getEntry("/ArmSubsystem/WristPosition").setDouble(m_wrist.getEncoder().getPosition());
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

}
