package com.maxtech.maxxi.subsystems;

import com.maxtech.maxxi.constants.MotorConstants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
    private final CANSparkMax intake = new CANSparkMax(MotorConstants.INTAKE_ID, MotorType.kBrushless);
    private double setIntakeSpeed = 0;
    private double prevIntakeSpeed = 0;

    public IntakeSubsystem() {
        intake.setIdleMode(CANSparkMax.IdleMode.kBrake);
        intake.setOpenLoopRampRate(0);
        intake.setSmartCurrentLimit(40);
    }

    public void setSpeed(Double speed) {
        setIntakeSpeed = speed;
    }


    @Override
    public void periodic() {
        if (prevIntakeSpeed == setIntakeSpeed)
            return;
        prevIntakeSpeed = setIntakeSpeed;
        intake.set(setIntakeSpeed);
    }
}
