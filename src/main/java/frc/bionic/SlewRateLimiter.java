// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.bionic;

import edu.wpi.first.wpiutil.WPIUtilJNI;

/**
 * A class that limits the rate of change of an input value. Useful for implementing voltage,
 * setpoint, and/or output ramps. A slew-rate limit is most appropriate when the quantity being
 * controlled is a velocity or a voltage; when controlling a position, consider using a {@link
 * edu.wpi.first.math.trajectory.TrapezoidProfile} instead.
 */
public class SlewRateLimiter {
  private final double rateLimitUp;
  private final double rateLimitDown;
  private double prevVal;
  private double prevTime;

  /**
   * Creates a new SlewRateLimiter with the given rate limit and initial value.
   *
   * @param rateLimit The rate-of-change limit, in units per second.
   * @param initialValue The initial value of the input.
   */
  public SlewRateLimiter(double rateLimitUp, double rateLimitDown, double initialValue) {
    this.rateLimitUp = rateLimitUp;
    this.rateLimitDown = rateLimitDown;

    prevVal = initialValue;
    prevTime = WPIUtilJNI.now() * 1e-6;
  }

  /**
   * Creates a new SlewRateLimiter with the given rate limit and an initial value of zero.
   *
   * @param rateLimit The rate-of-change limit, in units per second.
   */
  public SlewRateLimiter(double rateLimitUp, double rateLimitDown) {
    this(rateLimitUp, rateLimitDown, 0);
  }

  /**
   * Filters the input to limit its slew rate.
   *
   * @param input The input value whose slew rate is to be limited.
   * @return The filtered value, which will not change faster than the slew rate.
   */
  public double calculate(double input) {
    double currentTime = WPIUtilJNI.now() * 1e-6;
    double elapsedTime = currentTime - prevTime;

    double inputChange = input - prevVal;

    double minClampUp = -rateLimitUp * elapsedTime;
    double maxClampUp = rateLimitUp * elapsedTime;

    double minClampDown = -rateLimitDown * elapsedTime;
    double maxClampDown = rateLimitDown * elapsedTime;

    // If the change in input is higher than the last time (increasing)
    if(inputChange > 0){
      prevVal +=
        clamp(inputChange, minClampUp, maxClampUp);
    // If the change in input is lower than last time (decreasing)
    } else if (inputChange < 0){
      prevVal +=
        clamp(inputChange, minClampDown, maxClampDown);
    }

    prevTime = currentTime;
    return prevVal;
  }

  /**
   * Resets the slew rate limiter to the specified value; ignores the rate limit when doing so.
   *
   * @param value The value to reset to.
   */
  public void reset(double value) {
    prevVal = value;
    prevTime = WPIUtilJNI.now() * 1e-6;
  }

    /**
   * Returns value clamped between low and high boundaries.
   *
   * @param value Value to clamp.
   * @param low The lower boundary to which to clamp value.
   * @param high The higher boundary to which to clamp value.
   */
  public static double clamp(double value, double low, double high) {
    return Math.max(low, Math.min(value, high));
  }
}