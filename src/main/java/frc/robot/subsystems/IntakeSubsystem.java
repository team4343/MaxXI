package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.MotorConstants;

public class IntakeSubsystem extends SubsystemBase {
    public enum IntakeState {
        CONE_IN, CONE_OUT, CUBE_IN, CUBE_OUT, STOPPED
    }

    private final CANSparkMax intake = new CANSparkMax(MotorConstants.INTAKE_ID, MotorType.kBrushless);
    private IntakeState prev_intake_state = IntakeState.STOPPED;
    private IntakeState intake_state = IntakeState.STOPPED;

    public IntakeSubsystem() {
        intake.setIdleMode(CANSparkMax.IdleMode.kBrake);
        intake.setOpenLoopRampRate(0);
        intake.setSmartCurrentLimit(40);
    }

    public void setState(IntakeState state) {
        intake_state = state;
    }


    @Override
    public void periodic() {
        if (prev_intake_state.equals(intake_state))
            return;

        prev_intake_state = intake_state;
        double m_intakeSpeed = 1;
        switch (intake_state) {
            case CONE_IN:
            case CUBE_OUT:
                intake.set(m_intakeSpeed); break;
            case CONE_OUT:
            case CUBE_IN:
                intake.set(-m_intakeSpeed); break;
            case STOPPED:
                intake.set(0); break;
        }
    }
}
