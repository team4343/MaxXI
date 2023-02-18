package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import io.github.oblarg.oblog.Loggable;

public class IntakeSubsystem extends SubsystemBase implements Loggable {
    public enum IntakeState {
        CONE_IN, CONE_OUT, CUBE_IN, CUBE_OUT, STOPPED
    }

    private final CANSparkMax m_intake = new CANSparkMax(Constants.ArmConstants.INTAKE_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final double m_intakeSpeed = 0.5;
    private IntakeState m_intake_state = IntakeState.STOPPED;

    public IntakeSubsystem() {
        m_intake.setIdleMode(CANSparkMax.IdleMode.kBrake);
        m_intake.setOpenLoopRampRate(0);
        m_intake.setSmartCurrentLimit(40);
    }

    public void setState(IntakeState state) {
        m_intake_state = state;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
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
    private static class Position{
        public int intake = 0;
        public Position(int intake){
            this.intake = intake;
        }
    }

    private final Position Cone_In = new Position(0);
    private final Position Cone_Out = new Position(0);
    private final Position Cube_In = new Position(0);
    private final Position Cube_Out = new Position(0);
    private final Position Stopped = new Position(0);

}
