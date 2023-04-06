package com.maxtech.maxxi.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LightSubsystem extends SubsystemBase {
    enum State {
        Blank, // Blank
        Default, // Blue and yellow gradient
        Rainbow, // Rainbow
        Purple, // Purple
        Red, // Red
        RedBlinking, // Red Blinking
        Green, // Green
        GreenBlinking, // Green Blinking
        Blue, // Blue
        BlueBlinking, // Blue Blinking
        Yellow, // Yellow
        YellowBlinking, // Yellow Blinking
    }

    private State m_state = State.Purple;
    private final AddressableLED m_led = new AddressableLED(0);
    private final AddressableLEDBuffer m_buffer = new AddressableLEDBuffer(200);

    private final int lastBlink = 0;

    public LightSubsystem() {
        m_led.setLength(m_buffer.getLength());
        m_led.setData(m_buffer);
        m_led.start();
    }

    @Override
    public void periodic() {
        switch (m_state) {
            case Blank:
                setSolid(0, 0, 0);
            case Default:
                break;
            case Blue:
                blue();
                break;
            case Purple:
                if ((int)(Timer.getFPGATimestamp()*12) % 2 == 0)
                    setSolid(100, 0, 0);
                else
                    setSolid(0,0,0);
                break;


            case Rainbow:
                rainbow();
        }

        m_led.setData(m_buffer);
    }

    void setSolid(int r, int g, int b) {
        for (var i = 0; i < m_buffer.getLength(); i++) {
            m_buffer.setRGB(i, r, g, b);
        }
    }

    private int m_rainbowFirstPixelHue = 0;

    private void rainbow() {
        // For every pixel
        for (var i = 0; i < m_buffer.getLength(); i++) {
            // Calculate the hue - hue is easier for rainbows because the color
            // shape is a circle so only one value needs to precess
            final var hue = (m_rainbowFirstPixelHue + (i * 180 / m_buffer.getLength())) % 180;
            // Set the value
            m_buffer.setHSV(i, hue, 255, 128);
        }
        // Increase by to make the rainbow "move"
        m_rainbowFirstPixelHue += 3;
        // Check bounds
        m_rainbowFirstPixelHue %= 180;
    }

    private void red(){
        for (var i = 0; i < m_buffer.getLength(); i++){
            m_buffer.setRGB(i, 255, 0,0);
        }
    }
    private void blue(){
        for (var i = 0; i < m_buffer.getLength(); i++){
            m_buffer.setRGB(i, 0, 0,255);
        }
    }
    public void setState(State state) {
        this.m_state = state;
    }
}
