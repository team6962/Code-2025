package com.team6962.lib.utils;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;

/**
 * Utility class for interpreting CTRE Phoenix {@link StatusSignal}s and {@link StatusCode}s. This
 * class provides two methods for handling these objects:
 *
 * <ul>
 *   <li>{@link #check(StatusCode)} logs an error with a stack trace if the status code is not OK
 *   <li>{@link #unwrap(StatusSignal)} returns the value contained in a status signal, logging an
 *       error with a stack trace if the status code is not OK
 * </ul>
 */
public final class CTREUtils {
  private CTREUtils() {}

  public static void check(StatusCode code) {
    if (code.isOK()) return;

    System.err.println(
        "CTRE Status Code " + code.value + " " + code.getName() + ": " + code.getDescription());

    for (StackTraceElement element : Thread.currentThread().getStackTrace()) {
      System.err.println("\n\tat " + element);
    }
  }

  public static <T> T unwrap(StatusSignal<T> signal) {
    check(signal.getStatus());

    return signal.getValue();
  }
}
