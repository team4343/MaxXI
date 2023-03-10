package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import io.github.oblarg.oblog.Loggable;

public class IntakeSubsystem extends SubsystemBase implements Loggable {
    public enum IntakeState {
        CONE_IN, CONE_OUT, CUBE_IN, CUBE_OUT, STOPPED
    }

    private final CANSparkMax m_intake = new CANSparkMax(Constants.ArmConstants.INTAKE_ID, MotorType.kBrushless);
    private final CANSparkMax m_wrist = new CANSparkMax(Constants.ArmConstants.WRIST_ID, MotorType.kBrushless);

    private final double m_intakeSpeed = 1;
    private IntakeState m_prev_intake_state = IntakeState.STOPPED;
    private IntakeState m_intake_state = IntakeState.STOPPED;

    public IntakeSubsystem() {
        var tab = Shuffleboard.getTab("Intake");

        m_intake.setIdleMode(CANSparkMax.IdleMode.kBrake);
        m_intake.setOpenLoopRampRate(0);
        m_intake.setSmartCurrentLimit(40);

        m_wrist.restoreFactoryDefaults();
        m_wrist.setClosedLoopRampRate(1);
        m_wrist.setIdleMode(IdleMode.kBrake);

        m_wrist.getPIDController().setP(.1);
        m_wrist.getPIDController().setReference(18.5, ControlType.kPosition);

        tab.addNumber("Wrist position", () -> m_wrist.getEncoder().getPosition());
    }

    public void setState(IntakeState state) {
        m_intake_state = state;
    }

    public boolean hasGamePiece() {
        return false; // TODO
    }

    @Override
    public void periodic() {
        if (m_prev_intake_state.equals(m_intake_state)) {
            return;
        }        

        m_prev_intake_state = m_intake_state;

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
