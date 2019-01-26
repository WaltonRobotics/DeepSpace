package frc.robot;

import frc.utils.CrashDump;

public class CrashDumpTest {

    @Test
    public void crashDumpFileExistsTest() {
        CrashDump.logThrowableCrash(new Throwable("[IGNORE]: Crash dump file exists test."));

        File tmpDir = new File(CrashDump.CRASH_DUMP_DEFAULT_FILE_PATH);

        assertTrue(tmpDir.exists());
    }

}