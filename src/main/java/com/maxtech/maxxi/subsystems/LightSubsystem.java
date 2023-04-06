package com.maxtech.maxxi.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LightSubsystem extends SubsystemBase {
    public enum State {
        Blank, // Blank
        Default, // Blue and yellow gradient
        Rainbow, // Rainbow
        Purple, // Purple
        PurpleBlinking, // Purple Blinking
        Red, // Red
        RedBlinking, // Red Blinking
        Green, // Green
        GreenBlinking, // Green Blinking
        Blue, // Blue
        BlueBlinking, // Blue Blinking
        Yellow, // Yellow
        YellowBlinking, // Yellow Blinking
    }

    private State state = State.Blank;
    private final AddressableLED led = new AddressableLED(0);
    private final AddressableLEDBuffer buffer = new AddressableLEDBuffer(200);
    private double blinkTime = 0.25;
    private static final LightSubsystem instance = new LightSubsystem();

    public LightSubsystem() {
        if (instance != null)
            return; // Singleton
        led.setLength(buffer.getLength());
        led.setData(buffer);
        led.start();
    }

    @Override
    public void periodic() {
        switch (state) {
            case Blue:
                setSolid(0, 0, 100);
                break;
            case Red:
                setSolid(100, 0, 0);
                break;
            case RedBlinking:
                blink(100, 0, 0);
                break;
            case PurpleBlinking:
                blink(129, 0, 255);
                break;
            case Purple:
                setSolid(86, 0, 140);
                break;
            case YellowBlinking:
                blink(100, 100, 0);
                break;
            case GreenBlinking:
                blink(0, 100, 0);
                break;
            case Rainbow:
                rainbow();
                break;
            case Green:
                setSolid(0, 100, 0);
                break;
            case Yellow:
                setSolid(100, 100, 0);
                break;
            case BlueBlinking:
                blink(0, 0, 100);
                break;
            default:
                setSolid(0, 0, 0);
                break;
        }

        led.setData(buffer);
    }

    private void setSolid(int r, int g, int b) {
        for (var i = 0; i < buffer.getLength(); i++) {
            buffer.setRGB(i, r, g, b);
        }
    }

    private int m_rainbowFirstPixelHue = 0;

    private void blink(int r, int g, int b) {
        if ((int)(Timer.getFPGATimestamp()*6) % 2 == 0)
            setSolid(r, g, b);
        else
            setSolid(0,0,0);
    }

    private void rainbow() {
        // For every pixel
        for (var i = 0; i < buffer.getLength(); i++) {
            // Calculate the hue - hue is easier for rainbows because the color
            // shape is a circle so only one value needs to precess
            final var hue = (m_rainbowFirstPixelHue + (i * 180 / buffer.getLength())) % 180;
            // Set the value
            buffer.setHSV(i, hue, 255, 128);
        }
        // Increase by to make the rainbow "move"
        m_rainbowFirstPixelHue += 3;
        // Check bounds
        m_rainbowFirstPixelHue %= 180;
    }

    // idk Im drunk idk if this works. I think it does but tbh copliot did it.
    public void gradient(int r1, int g1, int b1, int r2, int g2, int b2) {
        for (var i = 0; i < buffer.getLength(); i++) {
            buffer.setRGB(i, (int) (r1 + (r2 - r1) * i / buffer.getLength()),
                    (int) (g1 + (g2 - g1) * i / buffer.getLength()),
                    (int) (b1 + (b2 - b1) * i / buffer.getLength()));
        }
    }

    public void setState(State state) {
        this.state = state;
    }

    public void setBlinkTime(double blinkTime) {
        this.blinkTime = blinkTime;
    }

    public static LightSubsystem getInstance() {
        return instance;
    }
}
