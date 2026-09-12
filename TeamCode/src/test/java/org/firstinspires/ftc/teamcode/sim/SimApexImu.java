package org.firstinspires.ftc.teamcode.sim;

import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.IMU;

import org.codeblooded.ftcodesim.hardware.drivetrain.SimulatedDrivetrain;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;

import java.lang.reflect.Array;
import java.lang.reflect.Proxy;

/** Creates the minimal drivetrain-backed IMU used by interactive tuner simulations. */
final class SimApexImu {
    private SimApexImu() { }

    /** Returns an IMU whose orientation follows the simulated drivetrain heading. */
    static IMU create(SimulatedDrivetrain drivetrain) {
        return (IMU) Proxy.newProxyInstance(SimApexImu.class.getClassLoader(),
                new Class<?>[] {IMU.class}, (proxy, method, args) -> {
                    if (method.getName().equals("getRobotOrientationAsQuaternion")) {
                        double halfHeading = drivetrain.position.theta / 2.0;
                        return new Quaternion((float) Math.cos(halfHeading), 0.0f, 0.0f,
                                (float) Math.sin(halfHeading), System.nanoTime());
                    }
                    if (method.getName().equals("getDeviceName")) {
                        return "Drivetrain-backed simulated IMU";
                    }
                    if (method.getName().equals("getManufacturer")) {
                        return HardwareDevice.Manufacturer.Other;
                    }
                    if (method.getName().equals("getVersion")) { return 1; }
                    if (method.getName().equals("getConnectionInfo")) { return "simulated"; }
                    return defaultValue(method.getReturnType());
                });
    }

    private static Object defaultValue(Class<?> type) {
        if (!type.isPrimitive()) {
            return type.isArray() ? Array.newInstance(type.getComponentType(), 0) : null;
        }
        if (type == boolean.class) { return false; }
        if (type == char.class) { return '\0'; }
        if (type == byte.class) { return (byte) 0; }
        if (type == short.class) { return (short) 0; }
        if (type == int.class) { return 0; }
        if (type == long.class) { return 0L; }
        if (type == float.class) { return 0.0f; }
        if (type == double.class) { return 0.0; }
        return null;
    }
}
