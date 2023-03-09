package frc.robot.subsystems;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SPI.Port;
import frc.robot.Robot;
import edu.wpi.first.wpilibj.SPI.Mode;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableListener;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableEntry;


public class arduino {
    private SPI spi;
    private Mode mode = Mode.kMode3;
    NetworkTableEntry team;
  
    int bitrate;
    byte data;
    byte[] buff = new byte[] {0};
    byte[] send = new byte[] {0,2}; // ignore 0 as it is needed in order to send data. Trying to fix so that is no longer so
    NetworkTableInstance inst = NetworkTableInstance.getDefault();

    public void spi_init() {
        spi = new SPI(Port.kOnboardCS0);
        spi.setClockRate(2000); 
        spi.setMode(mode);
        
    }
 
    public void spi_periodic() {
        get_team();
        spi.read(true, buff, 1);
        spi.write(send, 2);
    }
    
    void get_team(){
        
    NetworkTable table = inst.getTable("/FMSInfo");
    team = table.getEntry("IsRedAlliance");
    if (team.getBoolean(false) == true){
        send[1] = 20;
    }
    else{
        send[1] = 21;
    }

    }
}
