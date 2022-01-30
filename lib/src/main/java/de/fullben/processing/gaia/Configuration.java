package de.fullben.processing.gaia;

import static java.util.Objects.requireNonNull;

import de.fullben.processing.gaia.math.Vector3D;

/**
 * Class for storing configuration values which are used throughout the library.
 *
 * @author Benedikt Full
 */
public class Configuration {

  private static Configuration configuration = new Configuration(100.0, 0.0005, 0.05);
  private final double processingFactor;
  private final double sleepEpsilon;
  private final double maxFrameDuration;

  public Configuration(double processingFactor, double sleepEpsilon, double maxFrameDuration) {
    this.processingFactor = requireValidProcessingFactor(processingFactor);
    this.sleepEpsilon = requireValidSleepEpsilon(sleepEpsilon);
    this.maxFrameDuration = requireValidMaxFrameDuration(maxFrameDuration);
  }

  /**
   * Returns the current global configuration object used by the library.
   *
   * @return the current configuration
   * @see #update(Configuration)
   */
  public static Configuration current() {
    return configuration;
  }

  /**
   * Sets the given configuration as the global configuration object used by the library.
   *
   * @param config a non-{@code null} configuration
   * @see #current()
   */
  public static void update(Configuration config) {
    configuration = requireNonNull(config, "Configuration must not be null");
  }

  /**
   * Returns the factor with which any distance values should be multiplied with before being
   * returned when queried for usage in {@code Processing}. This is necessary, because in {@code
   * Gaia} the default unit for distances is meters, while {@code Processing}'s coordinate system is
   * based on pixels.
   *
   * <p>For bridging the gap between these coordinates systems, the {@code Vector3D} class offers a
   * set of so-called <i>Processing getters</i> ({@link Vector3D#x()}, {@link Vector3D#y()}, and
   * {@link Vector3D#z()}), which return the vector's values scaled based on the currently
   * configured <i>Processing factor</i>.
   *
   * <p>Assuming that this method returns {@code 100.0} and the following Vector exists:
   *
   * <pre>
   *   Vector3D v = new Vector3D(1.2, 1.0, 0.0);
   *   double x = v.getX(); // x == 1.2
   *   double xScaled = v.x(); // xScaled == 120
   * </pre>
   *
   * @return the factor with which to scale any meter values
   */
  public double getProcessingFactor() {
    return processingFactor;
  }

  /**
   * Returns the value which is used as a threshold to put rigid bodies to sleep. A body is put to
   * sleep whenever his linear and angular motion combined are lower than the sleep epsilon.
   *
   * @return the value of {@link #sleepEpsilon}
   */
  public double getSleepEpsilon() {
    return sleepEpsilon;
  }

  /**
   * Returns the maximum duration in seconds for which the physics may be updated in a rigid body or
   * particle world. If the value provided to a world's update routine exceeds this value, no update
   * will be performed.
   *
   * @return the value of {@link #maxFrameDuration}
   */
  public double getMaxFrameDuration() {
    return maxFrameDuration;
  }

  private static double requireValidProcessingFactor(double d) {
    if (d <= 0) {
      throw new IllegalArgumentException("Processing factor must be greater than zero");
    }
    return d;
  }

  private static double requireValidSleepEpsilon(double d) {
    if (d <= 0) {
      throw new IllegalArgumentException("Sleep epsilon must be greater than zero");
    }
    return d;
  }

  private static double requireValidMaxFrameDuration(double d) {
    if (d <= 0 || d > 0.2) {
      throw new IllegalArgumentException(
          "The maximum frame duration must be greater than 0 and no larger than 0.2");
    }
    return d;
  }
}
