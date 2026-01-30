package frc.robot.util;
import java.io.*;

public class BinaryFileLoader {

    private static final String FILE_PATH = "src\\main\\deploy\\fixed_width_data.bin";
    
    // Define record structure: double (8 bytes) + double (8 bytes) + double (8 bytes) = 24 bytes per record
    private static final int RECORD_SIZE = 24;

     /**
     * Writes fixed-width binary data to a file.
     */
    public static void writeBinaryFile() {
        try (FileOutputStream fos = new FileOutputStream(FILE_PATH);
             DataOutputStream dos = new DataOutputStream(fos)) {

            // Write 3 records
            dos.writeDouble(1.01); // 8 bytes
            dos.writeDouble(2.02); // 8 bytes
            dos.writeDouble(3.03); // 8 bytes

            dos.writeDouble(4.04); // 8 bytes
            dos.writeDouble(5.05); // 8 bytes
            dos.writeDouble(6.06); // 8 bytes

            dos.writeDouble(7.07); // 8 bytes
            dos.writeDouble(8.08); // 8 bytes
            dos.writeDouble(9.09); // 8 bytes

            System.out.println("Binary data written to " + FILE_PATH);

        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    /**
     * Reads fixed-width binary data from a file.
     */
    public static void readBinaryFile(int recordIndex) {
        // use 'r' mode for read-only access
        try (RandomAccessFile raf = new RandomAccessFile(FILE_PATH, "r")) {
            
            // Calculate and jump to the specific record position
            long offset = (long) recordIndex * RECORD_SIZE;
            raf.seek(offset); // Jump to byte 'offset'

            // Read fixed-width fields sequentially
            double entry1 = raf.readDouble();
            double entry2 = raf.readDouble();
            double entry3 = raf.readDouble();

            System.out.println("Record " + recordIndex + ": Entry1=" + entry1 + ", Entry2=" + entry2 + ", Entry3=" + entry3);
            
        } catch (IOException e) {
            e.printStackTrace();
        }
    }
}