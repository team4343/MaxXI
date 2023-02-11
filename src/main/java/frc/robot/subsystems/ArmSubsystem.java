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
        Pickup, Rest, PlacingA, PlacingB, PlacingC, PlacingD, PlacingE, // Possible System States
    }

    private final double elbow_Encoder_to_Degrees = 599.9;
    private final double shoulder_Encoder_to_Degrees = 342.8;

    // System motor controllers
    private final CANSparkMax m_shoulder = new CANSparkMax(ArmConstants.SHOULDER_ID, MotorType.kBrushless);
    private final CANSparkMax m_shoulderFollower = new CANSparkMax(ArmConstants.SHOULDER_FOLLOWER_ID, MotorType.kBrushless);
    private final CANSparkMax m_elbow = new CANSparkMax(ArmConstants.ELBOW_ID, MotorType.kBrushless);
    private final CANSparkMax m_intake = new CANSparkMax(ArmConstants.INTAKE_ID, MotorType.kBrushless);
    private State m_state = State.Rest; // Init State

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
    private final POS REST    = new POS(0, 0);
    private final POS PICKUP  = new POS(0, 0);
    private final POS PLACE_A = new POS(0, 0);
    private final POS PLACE_B = new POS(0, 0);
    private final POS PLACE_C = new POS(0, 0);
    private final POS PLACE_D = new POS(0, 0);
    private final POS PLACE_E = new POS(0, 0);

    // PID SLOT IDS
    // P. Proportional output to the error of the system
    // I. Sum of error over time. This increases output to counteract steady state error
    // D. Rate of change of error. This decreases output to counteract oscillation
    private final PID SHOULDER_DEFAULT = new PID(0.005, 0, 0, 0);
    private final PID SHOULDER_PICKUP  = new PID(0, 0, 0, 0);
    private final PID SHOULDER_A       = new PID(0, 0, 0, 0);
    private final PID SHOULDER_B       = new PID(0, 0, 0, 0);
    private final PID SHOULDER_C       = new PID(0, 0, 0, 0);
    private final PID SHOULDER_D       = new PID(0, 0, 0, 0);
    private final PID SHOULDER_E       = new PID(0, 0, 0, 0);

    private final PID ELBOW_DEFAULT = new PID(0, 0, 0, 0);
    private final PID ELBOW_PICKUP  = new PID(0, 0, 0, 0);
    private final PID ELBOW_A       = new PID(0, 0, 0, 0);
    private final PID ELBOW_B       = new PID(0, 0, 0, 0);
    private final PID ELBOW_C       = new PID(0, 0, 0, 0);
    private final PID ELBOW_D       = new PID(0, 0, 0, 0);
    private final PID ELBOW_E       = new PID(0, 0, 0, 0);


    public ArmSubsystem() {
        // Follow the main shoulder encoder, inverted.
        m_shoulderFollower.follow(m_shoulder, true);

        var tab = Shuffleboard.getTab("ArmSubsystem");
        tab.addString("State", () -> m_state.name());

        // ****** INITIALIZATION
        // Set the idle mode to brake
        m_shoulder.setIdleMode(CANSparkMax.IdleMode.kBrake);
        m_elbow.setIdleMode(CANSparkMax.IdleMode.kBrake);

        // Set the peak and nominal output
        m_shoulder.setSmartCurrentLimit(20);
        m_elbow.setSmartCurrentLimit(20);

        m_shoulder.setOpenLoopRampRate(0.25);
        m_elbow.setOpenLoopRampRate(0.25);

        // Set default PID values shoulder
        for (var config : new PID[]{SHOULDER_DEFAULT, SHOULDER_PICKUP, SHOULDER_A,
                SHOULDER_B, SHOULDER_C, SHOULDER_D, SHOULDER_E}) {
            m_shoulder.getPIDController().setP(config.P, config.slot);
            m_shoulder.getPIDController().setI(config.I, config.slot);
            m_shoulder.getPIDController().setD(config.D, config.slot);
            m_shoulder.getPIDController().setFF(config.FF, config.slot);
            m_shoulder.getPIDController().setDFilter(config.DF, config.slot);
        }

        // Set default PID values elbow
        for (var config : new PID[]{ELBOW_DEFAULT, ELBOW_PICKUP, ELBOW_A,
                ELBOW_B, ELBOW_C, ELBOW_D, ELBOW_E}) {
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
        if (m_state == state)
            return;

        // State is going to pickup from somewhere other than rest
        // Needs to go to rest first then can go to pickup.
        else if (state != State.Rest && m_state == State.Pickup)
            m_state = State.Rest;

        // State is in some drop-off [A,E]. It can go to any other drop off state or Rest.
        else if (m_state != State.Rest && m_state != State.Pickup)
            m_state = state;

        // In rest position it can go anywhere now.
        else
            m_state = state;
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
        switch (m_state) {
            case Rest: restPeriodic(); break;
            case Pickup: pickupPeriodic(); break;
            case PlacingA: placingAPeriodic(); break;
            case PlacingB: placingBPeriodic(); break;
            case PlacingC: placingCPeriodic(); break;
            case PlacingD: placingDPeriodic(); break;
            case PlacingE: placingEPeriodic(); break;
        }

        // Update the dashboard
        SmartDashboard.putNumber("Shoulder Position", m_shoulder.getEncoder().getPosition());
        SmartDashboard.putNumber("Elbow Position", m_elbow.getEncoder().getPosition());
    }

    private void pickupPeriodic() {
        // Turn the intake wheels on
        // Lower to pickup position+
        m_shoulder.getEncoder().setPosition(0);
        m_elbow.getEncoder().setPosition(0); // TODO: Set to pickup position

        m_intake.set(1);
    }

    private void restPeriodic() {
        // Turn off intake wheels
        // Go to rest position
        m_shoulder.getEncoder().setPosition(0);
        m_elbow.getEncoder().setPosition(0);

        m_intake.set(0);
    }

    private void placingAPeriodic() {
        // Go to this placing position
        // Turn intake on

        m_intake.set(-1);
    }

    private void placingBPeriodic() {
        // Go to this placing position
        // Turn intake on

        m_intake.set(-1);
    }

    private void placingCPeriodic() {
        // Go to this placing position
        // Turn intake on

        m_intake.set(-1);
    }

    private void placingDPeriodic() {
        // Go to this placing position
        // Turn intake on

        m_intake.set(-1);
    }

    private void placingEPeriodic() {
        // Go to this placing position
        // Turn intake on
        double new_angle = convert(35.0);
        // new_angle is encoder steps
        m_intake.set(-1);
    }

    private double  convert(double desired) {
        return desired * shoulder_Encoder_to_Degrees;
    }

}
