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

    public boolean hasGamePiece() {
        return false;
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
    private static class Intake_Speed {
        public int intake = 0;
        public Intake_Speed(int intake){
            this.intake = intake;
        }
    }

    private final Intake_Speed Cone_In = new Intake_Speed(0);
    private final Intake_Speed Cone_Out = new Intake_Speed(0);
    private final Intake_Speed Cube_In = new Intake_Speed(0);
    private final Intake_Speed Cube_Out = new Intake_Speed(0);
    private final Intake_Speed Stopped = new Intake_Speed(0);

}
