package frc.robot.util.led;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class SK6811RGBWBuffer {
    private byte[] buf;

    public SK6811RGBWBuffer(int length) {
        buf = new byte[(length * 5) + ((length * 5) % 4)];
    }

    public void fillRGBW(int r, int g, int b, int w) {
        byte[] one = {(byte) r, (byte) g, (byte) b, (byte) w};

        int stopCounter = 0;
        int lightIndex = 0;
        for (int i = 0; i < buf.length; i++) {
            if (stopCounter == 3) {
                buf[i] = 0;
                stopCounter = 0;
            } else {
                buf[i] = one[lightIndex];
                stopCounter++;
                lightIndex++;
                if (lightIndex == one.length) {
                    lightIndex = 0;
                }
            }
        }
    }
    
    public byte[] getBuf() {
        return buf;
    }

    public int getFakeLength() {
        return buf.length / 4;
    }
}