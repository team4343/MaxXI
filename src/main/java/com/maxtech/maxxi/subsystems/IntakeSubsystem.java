package com.maxtech.maxxi.subsystems;

import com.maxtech.maxxi.constants.MotorConstants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class IntakeSubsystem extends SubsystemBase {

    ArduinoSubsystem arduino = new ArduinoSubsystem();
    public enum State {
        DriverControl, ConeIn, ConeOut, Stopped
    }
    private final CANSparkMax intake = new CANSparkMax(MotorConstants.INTAKE_ID, MotorType.kBrushless);
    private double setIntakeSpeed = 0;
    private double prevIntakeSpeed = 0;
    private State state = State.DriverControl;

    public IntakeSubsystem() {
        intake.setIdleMode(CANSparkMax.IdleMode.kBrake);
        intake.setOpenLoopRampRate(0);
        intake.setSmartCurrentLimit(40);
    }

    public void setSpeed(Double speed) {
        this.state = State.DriverControl;
        setIntakeSpeed = speed;
    }

    public void setState(State state) {
        this.state = state;
        this.setIntakeSpeed = 0.75; // absolute value
    }

    @Override
    public void periodic() {
        // Ignore a double set.
        if (prevIntakeSpeed == setIntakeSpeed)
            return;

        // If we're using state
        if (state != State.DriverControl) {
            switch (state) {
                case ConeIn:
                    intake.set(setIntakeSpeed); // TODO Confirm
                    arduino.set_cone_intake_lights();
                    break;
                case ConeOut:
                    intake.set(-setIntakeSpeed);
                    arduino.set_cube_intake_lights();
                    break;
                default:
                    intake.set(0);
            }
        } else {
            prevIntakeSpeed = setIntakeSpeed;
            intake.set(setIntakeSpeed);
        }
    }

    public double getCurrent() {
        return intake.getOutputCurrent();
    }
}
