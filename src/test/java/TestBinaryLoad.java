import static org.junit.jupiter.api.Assertions.assertEquals;

import java.io.*;
import java.util.Arrays;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import frc.robot.util.BinaryLoader;

class TestBinaryLoader {
    BinaryLoader loader;

    @BeforeEach // this method will run before each test
    void setup() {
        loader = new BinaryLoader();
    }

    @Test
    void testReadRecord() throws IOException {
        assertEquals(3, loader.readRecord(1).length);
        System.out.println(Arrays.toString(loader.readRecord(1)));
    }
}