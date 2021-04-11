package de.fullben.processing.gaia.rigidbodies.fgens;

import de.fullben.processing.gaia.math.Matrix3;
import de.fullben.processing.gaia.math.Vector3D;
import de.fullben.processing.gaia.rigidbodies.RigidBody;

/**
 * A force generator with a control aerodynamic surface. This requires three inertia tensors, for
 * the two extremes (minimum and maximum) and resting position of the control surface. The latter
 * tensor is the one inherited from the base class, while the two extremes are stored in this class.
 *
 * @author Benedikt Full
 */
public class AeroControl extends Aero {

  /** The aerodynamic tensor for the surface when the control is at its minimum value. */
  private final Matrix3 minTensor;
  /** The aerodynamic tensor for the surface when the control is at its maximum value. */
  private final Matrix3 maxTensor;
  /**
   * The current position of the control for this surface. This should range between -1 (in which
   * case the {@link #minTensor} value is used), through 0 (where the tensor-value stored in the
   * base-{@link Aero} is used) to +1 (where the {@link #maxTensor} value is used).
   */
  private double controlSetting;

  /**
   * Constructs a new aerodynamic control surface with the given properties.
   *
   * @param base the base tensor
   * @param min the minimum tensor
   * @param max the maximum tensor
   * @param position the relative position of the aerodynamic surface in body coordinates
   * @param windSpeed the wind speed of the environment
   */
  public AeroControl(
      Matrix3 base, Matrix3 min, Matrix3 max, Vector3D position, Vector3D windSpeed) {
    super(base, position, windSpeed);
    minTensor = min;
    maxTensor = max;
  }

  @Override
  public Matrix3 getTensor() {
    if (controlSetting <= -1.0) {
      return minTensor;
    } else if (controlSetting >= 1.0) {
      return maxTensor;
    } else if (controlSetting < 0) {
      return Matrix3.linearInterpolate(minTensor, super.getTensor(), controlSetting + 1.0);
    } else if (controlSetting > 0) {
      return Matrix3.linearInterpolate(super.getTensor(), maxTensor, controlSetting);
    }
    return super.getTensor();
  }

  /**
   * Applies the force to the given rigid body.
   *
   * @param body the rigid body to which this method applies a force
   * @param duration the duration for which to apply the force in seconds
   */
  @Override
  public void updateForce(RigidBody body, double duration) {
    Matrix3 tensor = getTensor();
    updateForceFromTensor(body, duration, tensor);
  }

  /**
   * Returns the current control value
   *
   * @return value of {@link #controlSetting}
   */
  public double getControlSetting() {
    return controlSetting;
  }

  /**
   * Sets the control setting to the provided value.
   *
   * <p>This method has no effect if {@code -1 <= controlSetting <= 1} is not true.
   *
   * @param controlSetting the new control setting
   */
  public void setControlSetting(double controlSetting) {
    if (controlSetting >= -1 && controlSetting <= 1) {
      this.controlSetting = controlSetting;
    }
  }
}
