package core;

import org.junit.Test;
import java.nio.file.Files;
import java.io.File;
import static org.junit.Assert.*;

public class ApexStorageTest {
    @org.junit.Rule public org.junit.rules.TemporaryFolder folder = new org.junit.rules.TemporaryFolder();

    @Test public void preservesPreviousFileAndRecoversInterruptedReplacement() throws Exception {
        String previous = System.getProperty(ApexStorage.DIRECTORY_PROPERTY);
        try {
            System.setProperty(ApexStorage.DIRECTORY_PROPERTY, folder.getRoot().getAbsolutePath());
            ApexStorage.saveConstants("{\"old\":1}");
            ApexStorage.saveConstants("{\"new\":2}");
            File backup = new File(folder.getRoot(), "constants.json.bak");
            assertEquals("{\"old\":1}", new String(Files.readAllBytes(backup.toPath()), "UTF-8"));
            assertEquals("{\"new\":2}", new String(Files.readAllBytes(ApexStorage.getConstantsFile().toPath()), "UTF-8"));
            assertTrue(ApexStorage.getConstantsFile().delete());
            assertEquals(backup, ApexStorage.getReadableConstantsFile());
        } finally {
            if (previous == null) { System.clearProperty(ApexStorage.DIRECTORY_PROPERTY); }
            else { System.setProperty(ApexStorage.DIRECTORY_PROPERTY, previous); }
        }
    }

    @Test public void localizationUsesIndependentPrimaryAndBackupFiles() throws Exception {
        String previous = System.getProperty(ApexStorage.DIRECTORY_PROPERTY);
        try {
            System.setProperty(ApexStorage.DIRECTORY_PROPERTY, folder.getRoot().getAbsolutePath());
            ApexStorage.saveLocalization("{\"old\":1}");
            ApexStorage.saveLocalization("{\"new\":2}");
            File backup = new File(folder.getRoot(), "localization.json.bak");
            assertEquals("{\"old\":1}", new String(Files.readAllBytes(backup.toPath()), "UTF-8"));
            assertEquals("{\"new\":2}", new String(
                    Files.readAllBytes(ApexStorage.getLocalizationFile().toPath()), "UTF-8"));
            assertNotEquals(ApexStorage.getConstantsFile(), ApexStorage.getLocalizationFile());
        } finally {
            if (previous == null) { System.clearProperty(ApexStorage.DIRECTORY_PROPERTY); }
            else { System.setProperty(ApexStorage.DIRECTORY_PROPERTY, previous); }
        }
    }
}
