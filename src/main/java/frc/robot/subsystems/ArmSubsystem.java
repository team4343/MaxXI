package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import io.github.oblarg.oblog.Loggable;

public class ArmSubsystem extends SubsystemBase implements Loggable {
    public enum State {
        Pickup, Rest, PlacingA, PlacingB, PlacingC, PlacingD, PlacingE, Transit // Possible System States
    }

    private final double elbowPosToDegrees = 599.9;
    private final double shoulderPosToDegrees = 342.8;
    private final double stateTolerance = 0.5;

    // System motor controllers
    private final CANSparkMax m_shoulder = new CANSparkMax(ArmConstants.SHOULDER_ID, MotorType.kBrushless);
    private final CANSparkMax m_shoulderFollower = new CANSparkMax(ArmConstants.SHOULDER_FOLLOWER_ID, MotorType.kBrushless);
    private final CANSparkMax m_elbow = new CANSparkMax(ArmConstants.ELBOW_ID, MotorType.kBrushless);

    // System State - init in rest state
    private State m_state_desired = State.Rest;
    private State m_state_actual = State.Rest;

    // Class to hold position Data for each system state
    private static class POS {
        public int elbow, shoulder = 0;
        public POS(int elbow, int shoulder) {
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
        public PID(double P, double I, double D, double FF, double DF, int slot) {
            this.P = P;
            this.I = I;
            this.D = D;
            this.FF = FF;
            this.DF = DF;
            this.slot = slot;
        }
    }

    // Position Constants
    // Horizontal = 40, Down = 0, Up = 80
    private final POS REST      = new POS(0, 0);
    private final POS PICKUP    = new POS(4, 4);
    private final POS PLACING_A = new POS(0, 0);
    private final POS PLACING_B = new POS(0, 0);
    private final POS PLACING_C = new POS(0, 0);
    private final POS PLACING_D = new POS(0, 0);
    private final POS PLACING_E = new POS(0, 0);

    // PID SLOT IDS
    // P. Proportional output to the error of the system
    // I. Sum of error over time. This increases output to counteract steady state error
    // D. Rate of change of error. This decreases output to counteract oscillation
    private final PID SHOULDER_DEFAULT  = new PID(0.001, 0, 0, 0);
    private final PID SHOULDER_PICKUP   = new PID(0, 0, 0, 1);
    private final PID SHOULDER_PLACING  = new PID(0, 0, 0, 2);
    private final PID ELBOW_DEFAULT     = new PID(0.001, 0, 0, 0);
    private final PID ELBOW_PICKUP      = new PID(0, 0, 0, 1);
    private final PID ELBOW_PLACING     = new PID(0, 0, 0, 2);


    public ArmSubsystem() {
        // Follow the main shoulder encoder, inverted.
        m_shoulderFollower.follow(m_shoulder, true);

        var tab = Shuffleboard.getTab("ArmSubsystem");
        tab.addString("Desired State", () -> m_state_desired.name());
        tab.addString("Actual State", () -> m_state_actual.name());

        // ****** INITIALIZATION
        // Set the idle mode to brake
        m_shoulder.setIdleMode(CANSparkMax.IdleMode.kBrake);
        m_elbow.setIdleMode(CANSparkMax.IdleMode.kBrake);

        // Set the peak and nominal output
        m_shoulder.setSmartCurrentLimit(20);
        m_elbow.setSmartCurrentLimit(20);

        // Set the ramp rate of 0.25 seconds to full power
        m_shoulder.setOpenLoopRampRate(0.25);
        m_elbow.setOpenLoopRampRate(0.25);

        // Set default PID values for shoulder on all slots
        for (var config : new PID[]{SHOULDER_DEFAULT, SHOULDER_PICKUP, SHOULDER_PLACING}) {
            m_shoulder.getPIDController().setP(config.P, config.slot);
            m_shoulder.getPIDController().setI(config.I, config.slot);
            m_shoulder.getPIDController().setD(config.D, config.slot);
            m_shoulder.getPIDController().setFF(config.FF, config.slot);
            m_shoulder.getPIDController().setDFilter(config.DF, config.slot);
        }

        // Set default PID values elbow
        for (var config : new PID[]{ELBOW_DEFAULT, ELBOW_PICKUP, ELBOW_PLACING}) {
            m_elbow.getPIDController().setP(config.P, config.slot);
            m_elbow.getPIDController().setI(config.I, config.slot);
            m_elbow.getPIDController().setD(config.D, config.slot);
            m_elbow.getPIDController().setFF(config.FF, config.slot);
            m_elbow.getPIDController().setDFilter(config.DF, config.slot);
        }

        // Set the soft limits
        m_shoulder.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, 5);
        m_shoulder.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, -1);
        m_elbow.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, 5);
        m_elbow.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, -1);

    }

    public void setState(State state) {
        // Don't do anything already in correct state
        if (m_state_desired == state)
            return;

        switch (m_state_actual) {
            case Pickup:
                if (state == State.Rest)
                    m_state_desired = state;
                break;
            case Rest:
                m_state_desired = state;
                break;
            case PlacingA:
            case PlacingB:
            case PlacingC:
            case PlacingD:
            case PlacingE:
                if (state != State.Pickup)
                    m_state_desired = state;
                else
                    m_state_desired = State.Rest;
                break;
            case Transit:
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

        // If the state is not the same as the desired state, then run the periodic for that state.
        if (m_state_actual != m_state_desired) {
            switch (m_state_desired) {
                case Rest: restPeriodic(); break;
                case Pickup: pickupPeriodic(); break;
                case PlacingA: placingAPeriodic(); break;
                case PlacingB: placingBPeriodic(); break;
                case PlacingC: placingCPeriodic(); break;
                case PlacingD: placingDPeriodic(); break;
                case PlacingE: placingEPeriodic(); break;
            }
        }

        // Update the dashboard
        SmartDashboard.putNumber("Shoulder Position", m_shoulder.getEncoder().getPosition());
        SmartDashboard.putNumber("Elbow Position", m_elbow.getEncoder().getPosition());
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
    

    private void pickupPeriodic() {
        // Turn the intake wheels on
        // Lower to pickup position+
        m_elbow.getPIDController().setReference(PICKUP.elbow,
                CANSparkMax.ControlType.kPosition,
                ELBOW_DEFAULT.slot);

        m_shoulder.getPIDController().setReference(PICKUP.shoulder,
                CANSparkMax.ControlType.kPosition,
                SHOULDER_DEFAULT.slot);
    }

    private void restPeriodic() {
        // Turn off intake wheels
        // Go to rest position
        m_elbow.getPIDController().setReference(REST.elbow,
                CANSparkMax.ControlType.kPosition,
                ELBOW_DEFAULT.slot);

        m_shoulder.getPIDController().setReference(REST.shoulder,
                CANSparkMax.ControlType.kPosition,
                SHOULDER_DEFAULT.slot);
    }

    private void placingAPeriodic() {
        // Go to this placing position

    }

    private void placingBPeriodic() {
        // Go to this placing position
    }

    private void placingCPeriodic() {
        // Go to this placing position
    }

    private void placingDPeriodic() {
        // Go to this placing position
    }

    private void placingEPeriodic() {
        // Go to this placing position
    }

    private double  convert(double desired) {
        return desired * shoulderPosToDegrees;
    }

}
