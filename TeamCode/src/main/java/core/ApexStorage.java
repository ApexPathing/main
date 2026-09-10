package core;

import android.os.Environment;

import java.io.File;

/** Resolves the folder used for Apex Pathing's persisted tuning data. */
public final class ApexStorage {
    /** JVM property used by desktop tools such as FTCodeSim. */
    public static final String DIRECTORY_PROPERTY = "apexpathing.storageDirectory";

    private ApexStorage() { }

    public static File getDirectory() {
        String desktopDirectory = System.getProperty(DIRECTORY_PROPERTY);
        if (desktopDirectory != null && !desktopDirectory.trim().isEmpty()) {
            return new File(desktopDirectory);
        }

        // This remains the normal Robot Controller location on Android.
        return new File(Environment.getExternalStorageDirectory(), "FIRST/ApexPathing");
    }

    public static File getConstantsFile() {
        return new File(getDirectory(), "constants.json");
    }
    /** Write a synced temporary file, retaining the prior file for recovery. */
    public static synchronized void saveConstants(String json) throws java.io.IOException {
        File directory = getDirectory();
        if (!directory.isDirectory() && !directory.mkdirs()) {
            throw new java.io.IOException("Cannot create constants directory");
        }
        File target = getConstantsFile();
        File temporary = new File(directory, "constants.json.tmp");
        File backup = new File(directory, "constants.json.bak");
        try (java.io.FileOutputStream output = new java.io.FileOutputStream(temporary)) {
            output.write(json.getBytes(java.nio.charset.StandardCharsets.UTF_8));
            output.getFD().sync();
        }
        if (target.exists()) {
            if (backup.exists() && !backup.delete()) {
                throw new java.io.IOException("Cannot replace constants backup");
            }
            if (!target.renameTo(backup)) {
                throw new java.io.IOException("Cannot back up constants");
            }
        }
        if (!temporary.renameTo(target)) {
            if (backup.exists() && !backup.renameTo(target)) {
                throw new java.io.IOException("Save failed; recover constants.json.bak");
            }
            throw new java.io.IOException("Cannot install new constants");
        }
    }

    public static File getReadableConstantsFile() {
        File target = getConstantsFile();
        File backup = new File(getDirectory(), "constants.json.bak");
        return !target.exists() && backup.exists() ? backup : target;
    }
}
