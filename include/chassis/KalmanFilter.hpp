/*
    KalmanFilter.hpp

    Provides a small reusable Kalman filter template for smoothing measurements.
*/

#pragma once

template<typename T = double>
class KalmanFilter {
private:
  T estimate;
  T errorEstimate;
  T errorMeasure;
  T kalmanGain;
  bool initialized = false;

public:
  /**
   * @brief Creates a Kalman filter with initial estimate and error values.
   *
   * @param initialEstimate Starting estimate value.
   * @param initialError Starting estimate error.
   * @param measureError Measurement error used for updates.
   */
  KalmanFilter(T initialEstimate = 0, T initialError = 1, T measureError = 1)
    : estimate(initialEstimate),
      errorEstimate(initialError),
      errorMeasure(measureError),
      kalmanGain(0),
      initialized(false) {}

  /**
   * @brief Resets the filter to a new estimate.
   *
   * @param newEstimate Estimate to use after reset.
   */
  void reset(T newEstimate = 0) {
    estimate = newEstimate;
    errorEstimate = 1;
    kalmanGain = 0;
    initialized = false;
  }

  /**
   * @brief Updates the process and measurement error values.
   *
   * @param processNoise Error estimate used by the filter.
   * @param measurementNoise Measurement error used by the filter.
   */
  void setErrorValues(T processNoise, T measurementNoise) {
    errorEstimate = processNoise;
    errorMeasure = measurementNoise;
  }

  /**
   * @brief Filters a new measurement and returns the updated estimate.
   *
   * @param measurement New measured value.
   * @return Updated filtered estimate.
   */
  T update(T measurement) {
    if (!initialized) {
      estimate = measurement;
      initialized = true;
    }

    // Compute Kalman Gain
    kalmanGain = errorEstimate / (errorEstimate + errorMeasure);

    // Update estimate with measurement
    estimate = estimate + kalmanGain * (measurement - estimate);

    // Update the error estimate
    errorEstimate = (1 - kalmanGain) * errorEstimate;

    return estimate;
  }

  /**
   * @brief Gets the current filtered estimate.
   *
   * @return Current estimate value.
   */
  T getEstimate() const {
    return estimate;
  }
};
