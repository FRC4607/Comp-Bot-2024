package frc.robot.util.led;

public class SK6811RGBWBuffer {
    private byte[] buf;

    public SK6811RGBWBuffer(int length) {
        int calc = length * 5;
        calc += length / 3;
        calc = calc + (calc % 4);
        buf = new byte[calc];
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