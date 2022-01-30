package de.fullben.processing.gaia.rigidbodies.contacts;

import de.fullben.processing.gaia.math.Matrix4;
import de.fullben.processing.gaia.math.Vector3D;
import de.fullben.processing.gaia.rigidbodies.RigidBody;
import java.util.ArrayList;
import java.util.List;

/**
 * Wrapper class that holds the static routines for the generation of basic collision and bounding
 * geometry.
 *
 * @author Benedikt Full
 */
public class GeometryGenerator {

  private GeometryGenerator() {
    // Preventing instantiation
  }

  /**
   * Generates and returns the collision geometry for the provided mesh.
   *
   * <p>The current implementation of the method will always only generate one collision geometry
   * object for the provided entity, despite returning a list. This object may either be a sphere or
   * an axis aligned bounding box. The type is chosen based on which of the two shapes has the lower
   * volume when encompassing all vertices of the entity's mesh.
   *
   * @param rigidBody the rigid body associated with the {@code mesh}
   * @param mesh the mesh for which to generate collision geometry
   * @return a list containing one collision geometry object
   */
  public static List<CollisionGeometry> generateCollisionGeometry(RigidBody rigidBody, Mesh mesh) {
    List<CollisionGeometry> geometries = new ArrayList<>();
    CollisionSphere sphere = generateCollisionSphere(mesh.getVertices(), rigidBody);
    CollisionBox box = generateCollisionBox(mesh.getVertices(), rigidBody);
    if (sphere.getVolume() < box.getVolume()) {
      geometries.add(new CollisionSphere(rigidBody, sphere.getRadius()));
    } else {
      geometries.add(box);
    }
    return geometries;
  }

  /**
   * Generates and returns the collision sphere encompassing a sphere located at the position of the
   * given rigid body and with the given radius.
   *
   * @param rigidBody a rigid body representing a sphere
   * @param radius the radius of the sphere
   * @return a list containing one {@link CollisionSphere} object
   */
  public static List<CollisionGeometry> generateCollisionGeometry(
      RigidBody rigidBody, double radius) {
    List<CollisionGeometry> geometries = new ArrayList<>();
    geometries.add(new CollisionSphere(rigidBody, radius));
    return geometries;
  }

  /**
   * Generates and returns the collision box encompassing a cuboid located at the position of the
   * given rigid body and with the given dimensions.
   *
   * @param rigidBody a rigid body representing a cuboid
   * @param xDim the cuboid's width on the x axis
   * @param yDim the cuboid's width on the y axis
   * @param zDim the cuboid's width on the z axis
   * @return a list containing one {@link CollisionBox} object
   */
  public static List<CollisionGeometry> generateCollisionGeometry(
      RigidBody rigidBody, double xDim, double yDim, double zDim) {
    ArrayList<CollisionGeometry> geometries = new ArrayList<>();
    geometries.add(new CollisionBox(rigidBody, new Vector3D(xDim / 2.0, yDim / 2.0, zDim / 2.0)));
    return geometries;
  }

  /**
   * Generates and returns the bounding sphere for the provided collision geometry. The bounding
   * sphere will encompass all provided collision geometry objects.
   *
   * <p><b>Note:</b> The given entity is required to have valid collision geometry; this method will
   * return {@code null} if that is not the case.
   *
   * @param collisionGeometry an entity's collision geometry
   * @return a new bounding sphere or {@code null}
   */
  public static BoundingSphere generateBoundingSphere(List<CollisionGeometry> collisionGeometry) {
    List<CollisionSphere> spheres = generateCollisionSpheres(collisionGeometry);
    if (spheres.size() == 0) {
      return null;
    }
    // Use the properties of the first collision sphere to generate the initial bounding volume
    CollisionSphere csA = spheres.get(0);
    spheres.remove(csA);
    Vector3D offset =
        new Vector3D(csA.getOffset().getD(), csA.getOffset().getH(), csA.getOffset().getL());
    BoundingSphere bs = new BoundingSphere(csA.getRigidBody(), offset, csA.getRadius());
    for (CollisionSphere cs : spheres) {
      // Only check those volumes not already encompassed by the bounding sphere
      if (!boundingVolumeContainsCollisionVolume(bs, cs)) {
        Vector3D centerOffset = Vector3D.subtract(bs.getCenter(), cs.getCenter());
        double distSquared = centerOffset.squareMagnitude();
        double radiusDiff = cs.getRadius() - bs.getRadius();
        if (radiusDiff * radiusDiff >= distSquared && bs.getRadius() < cs.getRadius()) {
          // Bounding volume contains collision sphere
          bs.setCenter(cs.getCenter().getX(), cs.getCenter().getY(), cs.getCenter().getZ());
          bs.setRadius(cs.getRadius());
        } else {
          // Spheres are partially or not overlapping at all
          double distance = Math.sqrt(distSquared);
          double radius = (distance + bs.getRadius() + cs.getRadius()) * 0.5;
          // Move bounding sphere center towards collision sphere center by an amount proportional
          // to the sphere's radii
          if (distance > 0) {
            centerOffset.multiply(((radius - bs.getRadius()) / distance));
            bs.getCenter().add(centerOffset);
          }
        }
      }
    }
    return bs;
  }

  /**
   * Goes through the provided list of collision geometry objects and generates a collision sphere
   * for each collision box found in that list. The method returns a list containing all collision
   * spheres from the given list and the spheres generated.
   *
   * @param collisionGeometry a list of collision geometry objects
   * @return a list of collision spheres
   * @see #generateCollisionSphere(CollisionGeometry)
   */
  private static List<CollisionSphere> generateCollisionSpheres(
      List<CollisionGeometry> collisionGeometry) {
    List<CollisionSphere> spheres = new ArrayList<>();
    for (CollisionGeometry cg : collisionGeometry) {
      spheres.add(generateCollisionSphere(cg));
    }
    return spheres;
  }

  /**
   * Returns a collision sphere based on the given collision geometry object:
   *
   * <ul>
   *   <li>A collision box is provided - Generates and returns a new collision sphere encompassing
   *       the provided object
   *   <li>A collision sphere is provided - Returns a reference to the provided object
   * </ul>
   *
   * @param collisionGeometry a collision geometry object
   * @return the provided or a new collision sphere
   */
  private static CollisionSphere generateCollisionSphere(CollisionGeometry collisionGeometry) {
    if (collisionGeometry instanceof CollisionBox) {
      CollisionBox box = (CollisionBox) collisionGeometry;
      // Caching box properties
      Vector3D halfSize = box.getHalfSize();
      Vector3D center = box.getCenter();
      // Calculating one of the box's vertices
      Vector3D vertex =
          new Vector3D(
              center.getX() - halfSize.getX(),
              center.getY() - halfSize.getY(),
              center.getZ() - halfSize.getZ());
      // Radius is distance between center and one vertex
      double radius = Vector3D.subtract(center, vertex).magnitude();
      return new CollisionSphere(box.getRigidBody(), new Matrix4(box.getOffset()), radius);
    } else if (collisionGeometry instanceof CollisionSphere) {
      return (CollisionSphere) collisionGeometry;
    } else {
      throw new UnsupportedOperationException("Unknown collision geometry type");
    }
  }

  /**
   * Constructs a bounding sphere encompassing all provided vertices. The sphere will be associated
   * with the given rigid body.
   *
   * @param vertices the vertices of a 3D body
   * @param rigidBody the rigid body associated with the given vertices
   * @return a new sphere encompassing the provided vertices
   */
  private static CollisionSphere generateCollisionSphere(
      List<Vector3D> vertices, RigidBody rigidBody) {
    CollisionSphere sphere = new CollisionSphere(rigidBody, 0);
    Vector3D[] mostSepPoints = getMostSeparatePointsOnAABB(vertices);
    mostSepPoints[0].add(mostSepPoints[1]);
    mostSepPoints[0].multiply(0.5);
    sphere.setCenter(mostSepPoints[0].getX(), mostSepPoints[0].getY(), mostSepPoints[0].getZ());
    mostSepPoints[1].subtract(sphere.getCenter());
    sphere.setRadius(mostSepPoints[1].magnitude());
    for (Vector3D vertex : vertices) {
      includeVertex(vertex, sphere);
    }
    return sphere;
  }

  /**
   * Checks whether the given bounding sphere fully contains the given collision sphere.
   *
   * @param bSphere the bounding sphere
   * @param cSphere the collision sphere
   * @return true if the {@code bSphere} contains the {@code cSphere}
   */
  private static boolean boundingVolumeContainsCollisionVolume(
      BoundingSphere bSphere, CollisionSphere cSphere) {
    Vector3D centerOffset = Vector3D.subtract(bSphere.getCenter(), cSphere.getCenter());
    double distSquared = centerOffset.squareMagnitude();
    double radiusDiff = cSphere.getRadius() - bSphere.getRadius();
    // One of the two spheres contains the other
    if (radiusDiff * radiusDiff >= distSquared) {
      return bSphere.getRadius() > cSphere.getRadius();
    } else {
      return false;
    }
  }

  /**
   * Finds the two most separated points of the up to six points defining the axis aligned bounding
   * box (AABB) encompassing the provided vertices and returns copies of the two points.
   *
   * @param vertices the vertices of a 3D body
   * @return an array of the size 2, minimum point located at index 0, maximum at index 1
   */
  private static Vector3D[] getMostSeparatePointsOnAABB(List<Vector3D> vertices) {
    Vector3D[] points = new Vector3D[2];
    // Find the most extreme points along global axes
    int minX = 0, maxX = 0, minY = 0, maxY = 0, minZ = 0, maxZ = 0;
    for (int i = 0; i < vertices.size(); i++) {
      if (vertices.get(i).getX() < vertices.get(minX).getX()) {
        minX = i;
      }
      if (vertices.get(i).getX() > vertices.get(maxX).getX()) {
        maxX = i;
      }
      if (vertices.get(i).getY() < vertices.get(minY).getY()) {
        minY = i;
      }
      if (vertices.get(i).getY() > vertices.get(maxY).getY()) {
        maxY = i;
      }
      if (vertices.get(i).getZ() < vertices.get(minZ).getZ()) {
        minZ = i;
      }
      if (vertices.get(i).getZ() > vertices.get(maxZ).getZ()) {
        maxZ = i;
      }
    }
    // Compute square distances for the three pairs of points
    double dist2X = Vector3D.subtract(vertices.get(maxX), vertices.get(minX)).squareMagnitude();
    double dist2Y = Vector3D.subtract(vertices.get(maxY), vertices.get(minY)).squareMagnitude();
    double dist2Z = Vector3D.subtract(vertices.get(maxZ), vertices.get(minZ)).squareMagnitude();
    // Find most distant points
    int minIdx = minX;
    int maxIdx = maxX;
    if (dist2Y > dist2X && dist2Y > dist2Z) {
      minIdx = minY;
      maxIdx = maxY;
    }
    if (dist2Z > dist2X && dist2Z > dist2Y) {
      minIdx = minZ;
      maxIdx = maxZ;
    }
    // Copy most distant point pair to return array
    points[0] = new Vector3D(vertices.get(minIdx));
    points[1] = new Vector3D(vertices.get(maxIdx));
    return points;
  }

  /**
   * Moves and expands the bounding sphere to include the provided vertex. This method has no effect
   * if the provided vertex is located within the sphere already.
   *
   * @param vertex the point to be included
   * @param sphere the bounding sphere that will be altered to include the provided vertex
   */
  private static void includeVertex(Vector3D vertex, CollisionSphere sphere) {
    // Calculate square distance between sphere center and vertex
    Vector3D distance = Vector3D.subtract(vertex, sphere.getCenter());
    double squareDist = distance.squareMagnitude();
    double radius = sphere.getRadius();
    // Update sphere if point is outside of it
    if (squareDist > radius * radius) {
      double dist = Math.sqrt(squareDist);
      double newRadius = (radius + dist) * 0.5;
      double k = (newRadius - radius) / dist;
      sphere.setRadius(newRadius);
      distance.multiply(k);
      Vector3D center = sphere.getCenter();
      sphere.setCenter(
          center.getX() + distance.getX(),
          center.getY() + distance.getY(),
          center.getZ() + distance.getZ());
    }
  }

  /**
   * Constructs a axis aligned bounding box (AABB) encompassing all provided vertices. The AABB will
   * be associated with the given rigid body.
   *
   * @param vertices the vertices of a 3D body
   * @param rigidBody the rigid body associated with the given vertices
   * @return a new box encompassing the provided vertices
   */
  private static CollisionBox generateCollisionBox(List<Vector3D> vertices, RigidBody rigidBody) {
    Vector3D min;
    Vector3D max;
    Vector3D[] points = getMostSeparatePointsOnAABB(vertices);
    min = points[0];
    max = points[1];
    // Find center point
    Vector3D tmp = Vector3D.subtract(max, min);
    tmp.subtract(Vector3D.multiply(tmp, 0.5));
    Vector3D center = Vector3D.add(min, tmp);
    // Calculate dimensions
    double x = Math.abs(center.getX() - min.getX());
    double y = Math.abs(center.getY() - min.getY());
    double z = Math.abs(center.getZ() - min.getZ());
    return new CollisionBox(rigidBody, new Vector3D(x, y, z));
  }
}
