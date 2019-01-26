package frc.robot;

import java.io.ByteArrayOutputStream;
import java.io.OutputStream;
import frc.utils.Logger;

import org.junit.Test;

public class LoggerTest {

    @Test
    public void loggingInfoTest() {
        ByteArrayOutputStream os;

        List writeables = new ArrayList<OutputStream>();

        writeables.add(os);

        Logger logger = new Logger(writeables);

        String testLogString = "Logging info test.";

        logger.logInfo(testLogString);

        assertEquals(os.toString(), testLogString);
    }

}