package de.fullben.processing.gaia.rigidbodies.fgens;

import de.fullben.processing.gaia.math.Matrix3;
import de.fullben.processing.gaia.math.Quaternion;
import de.fullben.processing.gaia.math.Vector3D;

public class AngledAero extends Aero {

  Quaternion orientation;

  public AngledAero() {
    super(new Matrix3(), new Vector3D(), new Vector3D());
  }

  public void setOrientation(Quaternion orientation) {
    this.orientation = orientation;
  }
}
