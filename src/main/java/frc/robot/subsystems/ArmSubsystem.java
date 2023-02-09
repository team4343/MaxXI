package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import io.github.oblarg.oblog.Loggable;

public class ArmSubsystem extends SubsystemBase implements Loggable {
    public enum State {
        Pickup, Rest, PlacingA, PlacingB, PlacingC, PlacingD, PlacingE,
    }

    private final CANSparkMax m_shoulder =
            new CANSparkMax(ArmConstants.SHOULDER_ID, MotorType.kBrushless);
    private final CANSparkMax m_shoulderFollower =
            new CANSparkMax(ArmConstants.SHOULDER_FOLLOWER_ID, MotorType.kBrushless);
    private final CANSparkMax m_elbow =
            new CANSparkMax(ArmConstants.ELBOW_ID, MotorType.kBrushless);
    private final CANSparkMax m_intake =
            new CANSparkMax(ArmConstants.INTAKE_ID, MotorType.kBrushless);

    private State m_state = State.Rest;

    public ArmSubsystem() {
        // Follow the main shoulder encoder, inverted.
        m_shoulderFollower.follow(m_shoulder, true);

        var tab = Shuffleboard.getTab("ArmSubsystem");
        tab.addString("State", () -> m_state.name());
    }

    public void setState(State state) {
        m_state = state;
    }

    public ArmSubsystem.State deserializeStateFromString(String stateName) {
        System.out.println("Deserializing state for " + stateName);

        if (stateName.equals("Rest")) {
            return ArmSubsystem.State.Rest;
        } else if (stateName.equals("Pickup")) {
            return ArmSubsystem.State.Pickup;
        } else if (stateName.equals("PlacingA")) {
            return ArmSubsystem.State.PlacingA;
        } else if (stateName.equals("PlacingB")) {
            return ArmSubsystem.State.PlacingB;
        } else if (stateName.equals("PlacingC")) {
            return ArmSubsystem.State.PlacingC;
        } else if (stateName.equals("PlacingD")) {
            return ArmSubsystem.State.PlacingD;
        } else if (stateName.equals("PlacingE")) {
            return ArmSubsystem.State.PlacingE;
        }

        return ArmSubsystem.State.Rest;
    }

    public void periodic() {
        if (m_state.equals(State.Pickup))
            pickupPeriodic();
        else if (m_state.equals(State.Rest))
            restPeriodic();
        else if (m_state.equals(State.PlacingA))
            placingAPeriodic();
        else if (m_state.equals(State.PlacingB))
            placingBPeriodic();
        else if (m_state.equals(State.PlacingC))
            placingCPeriodic();
        else if (m_state.equals(State.PlacingD))
            placingDPeriodic();
        else if (m_state.equals(State.PlacingE))
            placingEPeriodic();
    }

    private void pickupPeriodic() {
        // Turn the intake wheels on
        // Lower to pickup position

        m_intake.set(1);
    }

    private void restPeriodic() {
        // Turn off intake wheels
        // Go to rest position

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

        m_intake.set(-1);
    }
}
