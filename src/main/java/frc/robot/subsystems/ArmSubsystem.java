package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import io.github.oblarg.oblog.Loggable;

public class ArmSubsystem extends SubsystemBase implements Loggable {
    public enum State {
        Pickup(0, 0), Rest(0, 0), PlacingA(Math.PI / 4, 0), PlacingB(0, 0), PlacingC(0,
                Math.PI / 2), PlacingD(Math.PI / 2, Math.PI / 2), PlacingE(0, 0);

        public final double elbow;
        public final double shoulder;

        private State(double elbow, double shoulder) {
            this.elbow = elbow;
            this.shoulder = shoulder;
        }
    }

    private State m_desiredState = State.Rest;

    // System motor controllers
    private final CANSparkMax m_shoulder =
            new CANSparkMax(ArmConstants.SHOULDER_ID, MotorType.kBrushless);
    private final CANSparkMax m_shoulderFollower =
            new CANSparkMax(ArmConstants.SHOULDER_FOLLOWER_ID, MotorType.kBrushless);
    private final CANSparkMax m_elbow =
            new CANSparkMax(ArmConstants.ELBOW_ID, MotorType.kBrushless);

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

    // PID SLOT IDS
    // P. Proportional output to the error of the system
    // I. Sum of error over time. This increases output to counteract steady state error
    // D. Rate of change of error. This decreases output to counteract oscillation
    private final PID SHOULDER_DEFAULT = new PID(0.025, 0, 0, 0);
    private final PID SHOULDER_STEADY = new PID(0.1, 0, 0, 1);
    private final PID ELBOW_DEFAULT = new PID(0.02, 0, 0, 0);
    private final PID ELBOW_STEADY = new PID(0.05, 0, 0, 1);

    public ArmSubsystem() {
        m_shoulder.restoreFactoryDefaults();
        m_shoulderFollower.restoreFactoryDefaults();
        m_elbow.restoreFactoryDefaults();

        m_elbow.setInverted(true);
        m_shoulder.setInverted(true);

        // Follow the main shoulder encoder, inverted.
        m_shoulderFollower.follow(m_shoulder, true);

        var tab = Shuffleboard.getTab("ArmSubsystem");
        tab.addString("State", () -> m_desiredState.name());

        tab.addNumber("Elbow position", () -> m_elbow.getEncoder().getPosition());
        tab.addNumber("Shoulder position", () -> m_shoulder.getEncoder().getPosition());

        tab.addNumber("Elbow setpoint position", () -> {
            return m_desiredState.elbow;
        });
        tab.addNumber("Shoulder setpoint position", () -> {
            return m_desiredState.shoulder;
        });

        tab.addBoolean("At state", this::atState);

        // Set the idle mode to brake
        m_shoulder.setIdleMode(CANSparkMax.IdleMode.kBrake);
        m_elbow.setIdleMode(CANSparkMax.IdleMode.kBrake);

        // Set the peak and nominal output
        m_shoulder.setSmartCurrentLimit(60);
        m_elbow.setSmartCurrentLimit(60);

        // Set the ramp rate of 2 seconds to full power
        m_shoulder.setClosedLoopRampRate(2);
        m_elbow.setClosedLoopRampRate(2);
        m_shoulder.setOpenLoopRampRate(2);
        m_elbow.setOpenLoopRampRate(2);

        // Set default PID values for shoulder on all slots
        for (var config : new PID[] {SHOULDER_DEFAULT, SHOULDER_STEADY}) {
            m_shoulder.getPIDController().setP(config.P, config.slot);
            m_shoulder.getPIDController().setI(config.I, config.slot);
            m_shoulder.getPIDController().setD(config.D, config.slot);
            m_shoulder.getPIDController().setFF(config.FF, config.slot);
            m_shoulder.getPIDController().setDFilter(config.DF, config.slot);
        }

        // Set default PID values elbow
        for (var config : new PID[] {ELBOW_DEFAULT, ELBOW_STEADY}) {
            m_elbow.getPIDController().setP(config.P, config.slot);
            m_elbow.getPIDController().setI(config.I, config.slot);
            m_elbow.getPIDController().setD(config.D, config.slot);
            m_elbow.getPIDController().setFF(config.FF, config.slot);
            m_elbow.getPIDController().setDFilter(config.DF, config.slot);
        }

        m_elbow.enableSoftLimit(SoftLimitDirection.kForward, true);
        m_elbow.enableSoftLimit(SoftLimitDirection.kReverse, true);
        m_shoulder.enableSoftLimit(SoftLimitDirection.kForward, true);
        m_shoulder.enableSoftLimit(SoftLimitDirection.kReverse, true);

        // Set the soft limits
        m_shoulder.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, 60);
        m_shoulder.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, -15);
        m_elbow.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, 60);
        m_elbow.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, -15);
    }

    public void setState(State desiredState) {
        this.m_desiredState = desiredState;
    }

    public boolean atState() {
        boolean shoulder_at_state = goalCalculator(m_shoulder.getEncoder().getPosition(), ArmConstants.SHOULDER_GEAR_RATIO(m_desiredState.shoulder));
        boolean elbow_at_state = goalCalculator(m_elbow.getEncoder().getPosition(), ArmConstants.ELBOW_GEAR_RATIO(m_desiredState.elbow));
        return shoulder_at_state && elbow_at_state;
    }

    private boolean goalCalculator(double current, double desired) {
        if (current == desired) return true;
        return Math.abs(current - desired) <= 10;
    }

    public ArmSubsystem.State deserializeStateFromString(String stateName) {
        System.out.println("Deserializing state for " + stateName);

        switch (stateName) {
            case "Rest":
                return State.Rest;
            case "Pickup":
                return State.Pickup;
            case "PlacingA":
                return State.PlacingA;
            case "PlacingB":
                return State.PlacingB;
            case "PlacingC":
                return State.PlacingC;
            case "PlacingD":
                return State.PlacingD;
            case "PlacingE":
                return State.PlacingE;
        }

        return ArmSubsystem.State.Rest;
    }

    public void periodic() {
        m_elbow.getPIDController().setReference(
                ArmConstants.ELBOW_GEAR_RATIO(m_desiredState.elbow), ControlType.kPosition);
        m_shoulder.getPIDController().setReference(
                ArmConstants.SHOULDER_GEAR_RATIO(m_desiredState.shoulder), ControlType.kPosition);
    }
}
