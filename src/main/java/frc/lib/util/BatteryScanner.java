package frc.lib.util;

import edu.wpi.first.wpilibj.SerialPort;
import java.util.Arrays;

public class BatteryScanner {
    public static final String defaultName = "0000-000";

    private static final int nameLength = 12;
    private static final byte[] scanCommand = new byte[] { 0x7e, 0x00, 0x08, 0x01, 0x00, 0x02, 0x01, (byte) 0xab,
            (byte) 0xcd };
    private static final byte[] responsePrefix = new byte[] { 0x02, 0x00, 0x00, 0x01, 0x00, 0x33, 0x31 };
    // private static final int fullResponseLength = responsePrefix.length +
    // nameLength;

    private static final byte endMark = 0x0d; // CR
    private static final int fullResponseLength = responsePrefix.length + nameLength + 1;

    // private static String name = defaultName;

    private static String name = defaultName;

    /**
     * Scans the battery. This should be called before the first loop cycle
     *
     * @param timeout The time to wait before giving up
     */
    public static String scanBattery(double timeout) {
        // Only scan on supported robots and in real mode

        try (SerialPort port = new SerialPort(9600, SerialPort.Port.kMXP)) {
            port.setTimeout(timeout);
            port.setWriteBufferSize(scanCommand.length);
            port.setReadBufferSize(fullResponseLength);

            port.write(scanCommand, scanCommand.length);
            byte[] response = port.read(fullResponseLength);

            // Ensure response is correct length
            if (response.length != fullResponseLength) {
                System.out.println("[BatteryTracker] Expected " + fullResponseLength
                        + " bytes from scanner, got " + response.length);
                return name;
            }

            // Ensure response starts with prefix
            for (int i = 0; i < responsePrefix.length; i++) {
                if (response[i] != responsePrefix[i]) {
                    System.out.println(
                            "[BatteryTracker] Invalid prefix from scanner.  Got data:");
                    System.out
                            .println("[BatteryTracker] " + Arrays.toString(response));
                    return name;
                }
            }

            // Ensure response ends with suffix
            if (response[response.length - 1] != endMark) {
                System.out.println("[BatteryTracker] Invalid suffix from scanner.  Got "
                                + response[response.length - 1]);
            }

            // Read name from data
            byte[] batteryNameBytes = new byte[nameLength];
            System.arraycopy(response, responsePrefix.length, batteryNameBytes, 0,
                    nameLength);
            name = new String(batteryNameBytes);
            System.out.println("[BatteryTracker] Scanned battery " + name);

        } catch (Exception e) {
            System.out.println(
                    "[BatteryTracker] Exception while trying to scan battery");
            e.printStackTrace();
        }

        return name;
    }

    /** Returns the name of the last scanned battery. */
    public static String getName() {
        return name;
    }
}