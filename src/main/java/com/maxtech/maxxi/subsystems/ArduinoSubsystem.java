package com.maxtech.maxxi.subsystems;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.SPI.Mode;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.Loggable;

public class ArduinoSubsystem extends SubsystemBase implements Loggable {
    private final SPI spi;
    public boolean match_started = false;
    NetworkTableEntry team;
    byte[] buff = new byte[] {0};
    byte[] send = new byte[] {0,2};
    NetworkTableInstance nt_handle = NetworkTableInstance.getDefault();

    public ArduinoSubsystem() {
        spi = new SPI(Port.kOnboardCS0);
        spi.setClockRate(2000);
        spi.setMode(Mode.kMode3);
    }

    @Override
    public void periodic() {
        get_team();
        spi.read(true, buff, 1);
        spi.write(send, 2);
    }

    public void get_team() {
        team = nt_handle.getTable("/FMSInfo").getEntry("IsRedAlliance");
        if (!match_started)
            send[1] = 22; // Idle
        else if (team.getBoolean(false))
            send[1] = 20; // Red team
        else
            send[1] = 21; // Blue team
    }
    public void set_cone_intake_lights() {
        send[1] = 23;
    }
    public void set_cube_intake_lights() {
        send[1] = 24;
    }
}
