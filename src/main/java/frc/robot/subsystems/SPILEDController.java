package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class SPILEDController extends AddressableLED {

    private SPI _SPI;

    public SPILEDController(int port) {
        super(port);
        _SPI = new SPI(Port.kOnboardCS0);
    }

    @Override
    public void close() {
        super.close();
        _SPI.close();
    }

    @Override
    public void setData(AddressableLEDBuffer buffer) {
        Color8Bit color = buffer.getLED8Bit(0);
        _SPI.write(new byte[] { (byte) color.red, (byte) color.green, (byte) color.blue }, 3);
    }

}
