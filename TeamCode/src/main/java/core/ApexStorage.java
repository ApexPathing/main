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

    /** Returns the localization calibration file used by the localization tuner. */
    public static File getLocalizationFile() {
        return new File(getDirectory(), "localization.json");
    }
    /** Write a synced temporary file, retaining the prior file for recovery. */
    public static synchronized void saveConstants(String json) throws java.io.IOException {
        saveJson(getConstantsFile(), json);
    }

    /** Atomically saves localization calibration while retaining its previous version. */
    public static synchronized void saveLocalization(String json) throws java.io.IOException {
        saveJson(getLocalizationFile(), json);
    }

    private static void saveJson(File target, String json) throws java.io.IOException {
        File directory = getDirectory();
        if (!directory.isDirectory() && !directory.mkdirs()) {
            throw new java.io.IOException("Cannot create tuning-data directory");
        }
        File temporary = new File(directory, target.getName() + ".tmp");
        File backup = new File(directory, target.getName() + ".bak");
        try (java.io.FileOutputStream output = new java.io.FileOutputStream(temporary)) {
            output.write(json.getBytes(java.nio.charset.StandardCharsets.UTF_8));
            output.getFD().sync();
        }
        if (target.exists()) {
            if (backup.exists() && !backup.delete()) {
                throw new java.io.IOException("Cannot replace " + target.getName() + " backup");
            }
            if (!target.renameTo(backup)) {
                throw new java.io.IOException("Cannot back up " + target.getName());
            }
        }
        if (!temporary.renameTo(target)) {
            if (backup.exists() && !backup.renameTo(target)) {
                throw new java.io.IOException("Save failed; recover " + target.getName() + ".bak");
            }
            throw new java.io.IOException("Cannot install " + target.getName());
        }
    }

    public static File getReadableConstantsFile() {
        return readableFile(getConstantsFile());
    }

    /** Returns the primary localization file, or its backup after an interrupted replacement. */
    public static File getReadableLocalizationFile() {
        return readableFile(getLocalizationFile());
    }

    private static File readableFile(File target) {
        File backup = new File(getDirectory(), target.getName() + ".bak");
        return !target.exists() && backup.exists() ? backup : target;
    }
}
