package com.maxtech.swervelib.parser.json;

import com.maxtech.swervelib.SwerveModule;
import com.maxtech.swervelib.parser.PIDFConfig;

/**
 * {@link SwerveModule} PID with Feedforward for the drive motor and angle motor.
 */
public class PIDFPropertiesJson
{

  /**
   * The PIDF with Integral Zone used for the drive motor.
   */
  public PIDFConfig drive;
  /**
   * The PIDF with Integral Zone used for the angle motor.
   */
  public PIDFConfig angle;
}
