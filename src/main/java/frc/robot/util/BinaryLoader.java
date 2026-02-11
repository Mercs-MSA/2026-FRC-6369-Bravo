package frc.robot.util;
import java.io.*;

import com.ctre.phoenix6.Utils;


public class BinaryLoader {
    // Define record structure: double (4 bytes) + double (4 bytes) + double (4 bytes) = 12 bytes per record
    private static final int RECORD_SIZE = 12;

    private RandomAccessFile raf;
    public long headerSize;


    public BinaryLoader() {
        try {
            // use 'r' mode for read-only access
            raf = new RandomAccessFile(Utils.isSimulation() ? "src/main/deploy/ShooterCalculations.bin" : "/home/lvuser/deploy/ShooterCalculations.bin", "r");
        } catch (IOException e) {
            e.printStackTrace();
        }

        try {
            raf.seek(0);
            headerSize = raf.readInt() & 0xFFFFFFFFL;
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    /**
     * Reads fixed-width binary data from a file.
     */
    public float[] readRecord(int recordIndex) throws IOException {
        // Calculate and jump to the specific record position
        long offset = (long) recordIndex * RECORD_SIZE + headerSize;
        raf.seek(offset); // Jump to byte 'offset'

        // Read fixed-width fields sequentially
        float entry1 = raf.readFloat();
        float entry2 = raf.readFloat();
        float entry3 = raf.readFloat();

        return new float[]{entry1, entry2, entry3};
    }
}
// radial tangent target
