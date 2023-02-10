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

    private final CANSparkMax m_shoulder = new CANSparkMax(ArmConstants.SHOULDER_ID, MotorType.kBrushless);
    private final CANSparkMax m_shoulderFollower = new CANSparkMax(ArmConstants.SHOULDER_FOLLOWER_ID, MotorType.kBrushless);
    private final CANSparkMax m_elbow = new CANSparkMax(ArmConstants.ELBOW_ID, MotorType.kBrushless);
    private final CANSparkMax m_intake = new CANSparkMax(ArmConstants.INTAKE_ID, MotorType.kBrushless);

    private final double elbow_Encoder_to_Degrees = 599.9;
    private final double shoulder_Encoder_to_Degrees = 342.8;

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

    public void periodic() {
        if (m_state == State.Pickup)
            pickupPeriodic();
        else if (m_state == State.Rest)
            restPeriodic();
        else if (m_state == State.PlacingA)
            placingAPeriodic();
        else if (m_state == State.PlacingB)
            placingBPeriodic();
        else if (m_state == State.PlacingC)
            placingCPeriodic();
        else if (m_state == State.PlacingD)
            placingDPeriodic();
        else if (m_state == State.PlacingE)
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
        double new_angle = convert(35.0);
        // new_angle is encoder steps
        m_intake.set(-1);
    }

    private double  convert(double desired) {
        return desired * shoulder_Encoder_to_Degrees;
    }

}
