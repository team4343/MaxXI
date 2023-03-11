package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.MotorConstants;
import io.github.oblarg.oblog.Loggable;

public class IntakeSubsystem extends SubsystemBase implements Loggable {
    public enum IntakeState {
        CONE_IN, CONE_OUT, CUBE_IN, CUBE_OUT, STOPPED
    }

    private final CANSparkMax m_intake = new CANSparkMax(MotorConstants.ArmConstants.INTAKE_ID, MotorType.kBrushless);
    private final double m_intakeSpeed = 1;
    private IntakeState m_prev_intake_state = IntakeState.STOPPED;
    private IntakeState m_intake_state = IntakeState.STOPPED;

    public IntakeSubsystem() {
        m_intake.setIdleMode(CANSparkMax.IdleMode.kBrake);
        m_intake.setOpenLoopRampRate(0);
        m_intake.setSmartCurrentLimit(40);
    }

    public void setState(IntakeState state) {
        m_intake_state = state;
    }

    public boolean hasGamePiece() {
        return false; // TODO
    }

    @Override
    public void periodic() {
        if (m_prev_intake_state.equals(m_intake_state))
            return;

        m_prev_intake_state = m_intake_state;
        switch (m_intake_state) {
            case CONE_IN:
            case CUBE_OUT:
                m_intake.set(m_intakeSpeed); break;
            case CONE_OUT:
            case CUBE_IN:
                m_intake.set(-m_intakeSpeed); break;
            case STOPPED:
                m_intake.set(0); break;
        }
    }
}
