package com.maxtech.maxxi.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LightSubsystem extends SubsystemBase {
    enum State {
        Default, // Blue and yellow gradient
    }

    private State m_state = State.Default;
    private final AddressableLED m_led = new AddressableLED(0);

    public LightSubsystem() {
        var m_ledBuffer = new AddressableLEDBuffer(100);

        m_led.setLength(m_ledBuffer.getLength());
        m_led.setData(m_ledBuffer);

        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
            // Sets the specified LED to the RGB values for red
            m_ledBuffer.setRGB(i, 255, 0, 0);
         }
         
         m_led.setData(m_ledBuffer);

        m_led.start();
    }
}
