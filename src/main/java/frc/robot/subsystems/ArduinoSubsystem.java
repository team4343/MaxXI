package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SPI.Mode;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.Loggable;


public class ArduinoSubsystem extends SubsystemBase implements Loggable {
    private SPI spi;
    private final Mode mode = Mode.kMode3;
    NetworkTableEntry team;

    byte[] buff = new byte[] {0};
    byte[] send = new byte[] {0,2}; // ignore 0 as it is needed in order to send data. Trying to fix so that is no longer so
    NetworkTableInstance nt_handle = NetworkTableInstance.getDefault();

    public ArduinoSubsystem() {
        spi = new SPI(Port.kOnboardCS0);
        spi.setClockRate(2000); 
        spi.setMode(mode);
    }

    @Override
    public void periodic() {
        get_team();
        spi.read(true, buff, 1);
        spi.write(send, 2);
    }
    
    void get_team() {
        team = nt_handle.getTable("/FMSInfo").getEntry("IsRedAlliance");
        if (team.getBoolean(false))
            send[1] = 20;
        else
            send[1] = 21;

    }
}
