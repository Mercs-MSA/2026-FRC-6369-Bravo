
import static org.junit.jupiter.api.Assertions.assertEquals;

import java.io.*;
import java.util.Arrays;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import frc.robot.math.ShooterMathProvider;


class TestBinaryLoader {
    @BeforeEach // this method will run before each test
    void setup() {
    }

    @Test
    void testSearch() throws IOException {
        double[] testArray = {1.0, 2.0, 3.0, 4.0, 5.0};
        int[] result = ShooterMathProvider.searchInput(3.5, testArray);
        assertEquals(3, result[0]);
        assertEquals(2, result[1]);
    }
}
