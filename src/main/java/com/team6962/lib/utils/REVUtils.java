package com.team6962.lib.utils;

import com.revrobotics.REVLibError;

public final class REVUtils {
    private REVUtils() {
    }

    public static void check(REVLibError error) {
        if (error == REVLibError.kOk) return;

        System.err.println("REV Error " + error.value + " " + error.toString());

        for (StackTraceElement element : Thread.currentThread().getStackTrace()) {
            System.err.println("\n\tat " + element);
        }
    }
}
