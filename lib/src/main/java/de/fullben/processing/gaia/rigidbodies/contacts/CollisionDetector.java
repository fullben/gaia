package de.fullben.processing.gaia.rigidbodies.contacts;

import de.fullben.processing.gaia.math.Vector3D;
import java.util.List;

/**
 * Wrapper class that holds the static fine grained collision detection routines.
 *
 * <p>Each of the methods has the same format: it takes the details of two objects and a pointer to
 * a contact registry to which to add any detected contacts. All methods return the number of
 * contacts they found.
 *
 * @author Benedikt Full
 */
public class CollisionDetector {

  private CollisionDetector() {
    // Preventing instantiation
  }

  /**
   * Detects all collisions between the geometry objects found in the two provided lists. Collisions
   * between objects from the same list are ignored.
   *
   * <p>If a collision is detected, this method will generate a corresponding {@link Contact} object
   * and add it to the provided {@link CollisionData}.
   *
   * @param geometryA the collision geometry representing one rigid body
   * @param geometryB the collision geometry representing another rigid body
   * @param data the data structure all detected collision will be written to
   * @return the number of collisions found
   */
  public static int detect(
      List<CollisionGeometry> geometryA, List<CollisionGeometry> geometryB, CollisionData data) {
    int collisions = 0;
    for (CollisionGeometry cgA : geometryA) {
      if (cgA instanceof CollisionSphere) {
        collisions += detect((CollisionSphere) cgA, geometryB, data);
      } else if (cgA instanceof CollisionBox) {
        collisions += detect((CollisionBox) cgA, geometryB, data);
      }
    }
    return collisions;
  }

  /**
   * Detects any collisions between the given collision sphere and any collision geometry object
   * found in the provided list.
   *
   * <p>If a collision is detected, this method will generate a corresponding {@link Contact} object
   * and add it to the provided {@link CollisionData}.
   *
   * @param sphere a collision sphere (partially) representing a rigid body
   * @param geometry the collision geometry representing another rigid body
   * @param data the data structure all detected collision will be written to
   * @return the number of collisions found
   */
  private static int detect(
      CollisionSphere sphere, List<CollisionGeometry> geometry, CollisionData data) {
    int collisions = 0;
    for (CollisionGeometry cg : geometry) {
      if (cg instanceof CollisionSphere) {
        collisions += sphereAndSphere(sphere, (CollisionSphere) cg, data);
      } else if (cg instanceof CollisionBox) {
        collisions += boxAndSphere((CollisionBox) cg, sphere, data);
      }
    }
    return collisions;
  }

  /**
   * Detects any collisions between the given collision box and any collision geometry object found
   * in the provided list.
   *
   * <p>If a collision is detected, this method will generate a corresponding {@link Contact} object
   * and add it to the provided {@link CollisionData}.
   *
   * @param box a collision box (partially) representing a rigid body
   * @param geometry the collision geometry representing another rigid body
   * @param data the data structure all detected collision will be written to
   * @return the number of collisions found
   */
  private static int detect(
      CollisionBox box, List<CollisionGeometry> geometry, CollisionData data) {
    int collisions = 0;
    for (CollisionGeometry cg : geometry) {
      if (cg instanceof CollisionSphere) {
        collisions += boxAndSphere(box, (CollisionSphere) cg, data);
      } else if (cg instanceof CollisionBox) {
        collisions += boxAndBox(box, (CollisionBox) cg, data);
      }
    }
    return collisions;
  }

  /**
   * Detects any collisions between the given plane (which is treated as a half space) and any
   * collision geometry object found in the provided list. The plane should represent an immovable
   * piece of world geometry.
   *
   * @param halfSpace a collision plane representing immovable world geometry
   * @param geometry the collision geometry representing another rigid body
   * @param data the data structure all detected collision will be written to
   * @return the number of collisions found
   */
  public static int detect(
      CollisionPlane halfSpace, List<CollisionGeometry> geometry, CollisionData data) {
    int collisions = 0;
    for (CollisionGeometry cg : geometry) {
      if (cg instanceof CollisionSphere) {
        collisions += sphereAndHalfSpace((CollisionSphere) cg, halfSpace, data);
      } else if (cg instanceof CollisionBox) {
        collisions += boxAndHalfSpace((CollisionBox) cg, halfSpace, data);
      }
    }
    return collisions;
  }

  /**
   * Checks whether the provided sphere and plane are colliding. The method assumes that the
   * provided plane is a half-space, which means that the entire back of the plane is treated as one
   * solid object. Consequently, any objects on the back side of the half-space will have their
   * contact normal in the opposite direction to the half-space normal. Any contact found by this
   * method will be added to the provided {@link CollisionData} object.
   *
   * @param sphere the sphere potentially involved in a collision
   * @param halfSpace the half-space potentially involved in a collision
   * @param data contact registry to which any found contact will be added
   * @return the number of contacts generated, either zero or one
   */
  private static int sphereAndHalfSpace(
      CollisionSphere sphere, CollisionPlane halfSpace, CollisionData data) {
    // Cache the sphere position
    Vector3D position = sphere.getAxis(3);
    // Find distance from plane
    double ballDistance =
        halfSpace.getDirection().scalarProduct(position)
            - sphere.getRadius()
            - halfSpace.getOriginOffset();
    if (ballDistance >= 0) {
      return 0;
    }
    // Create the contact - it has a normal in the plane direction
    Contact contact = new Contact();
    contact.setContactNormal(new Vector3D(halfSpace.getDirection()));
    contact.setPenetration(-ballDistance);
    Vector3D subtrahend = new Vector3D(halfSpace.getDirection());
    subtrahend.multiply(ballDistance + sphere.getRadius());
    position.subtract(subtrahend);
    contact.setContactPoint(position);
    contact.setBodyData(sphere.getRigidBody(), null, data.getFriction(), data.getRestitution());
    data.add(contact);
    return 1;
  }

  /**
   * Checks whether the provided sphere and plane are colliding. Any contact found by this method
   * will be added to the provided {@link CollisionData} object.
   *
   * @param sphere the sphere potentially involved in a collision
   * @param plane the plane potentially involved in a collision
   * @param data contact registry to which any found contact will be added
   * @return the number of contacts generated, either zero or one
   */
  private static int sphereAndPlane(
      CollisionSphere sphere, CollisionPlane plane, CollisionData data) {
    // Cache sphere position
    Vector3D position = sphere.getAxis(3);
    // Find distance from plane
    double centerDistance = plane.getDirection().scalarProduct(position) - plane.getOriginOffset();
    // Check if we're within the radius
    if (centerDistance * centerDistance > sphere.getRadius() * sphere.getRadius()) {
      return 0;
    }
    // Check which side of the plane we're on
    Vector3D normal = new Vector3D(plane.getDirection());
    double penetration = -centerDistance;
    if (centerDistance < 0) {
      normal.multiply(-1);
      penetration = -penetration;
    }
    penetration += sphere.getRadius();
    // Create the contact - it has a normal in the plane direction
    Contact contact = new Contact();
    contact.setContactNormal(normal);
    contact.setPenetration(penetration);
    Vector3D direction = new Vector3D(plane.getDirection());
    direction.multiply(centerDistance);
    position.subtract(direction);
    contact.setContactPoint(position);
    contact.setBodyData(sphere.getRigidBody(), null, data.getFriction(), data.getRestitution());
    data.add(contact);
    return 1;
  }

  /**
   * Checks whether the two provided sphere's are colliding. Two sphere's are in contact if the
   * distance between their centers is less than the sum of their radii. If this is the case, the
   * method will generate a new {@link Contact} and add it to the provided {@link CollisionData}
   * object.
   *
   * @param sphereA the first of the two spheres potentially involved in a collision
   * @param sphereB the second of the two spheres potentially involved in a collision
   * @param data contact registry to which any found contact will be added
   * @return the number of contacts generated, either zero or one
   */
  private static int sphereAndSphere(
      CollisionSphere sphereA, CollisionSphere sphereB, CollisionData data) {
    // Cache sphere positions
    Vector3D positionA = sphereA.getAxis(3);
    Vector3D positionB = sphereB.getAxis(3);
    // Find vector between spheres
    Vector3D midLine = new Vector3D(positionA);
    midLine.subtract(positionB);
    double size = midLine.magnitude();
    // See if the spheres are actually colliding
    if (size <= 0.0 || size >= sphereA.getRadius() + sphereB.getRadius()) {
      return 0;
    }
    // Create normal based on size
    Vector3D normal = Vector3D.multiply(midLine, 1.0 / size);
    Contact contact = new Contact();
    contact.setContactNormal(normal);
    midLine.multiply(0.5);
    positionA.add(midLine);
    contact.setContactPoint(positionA);
    contact.setPenetration(sphereA.getRadius() + sphereB.getRadius() - size);
    contact.setBodyData(
        sphereA.getRigidBody(), sphereB.getRigidBody(), data.getFriction(), data.getRestitution());
    data.add(contact);
    return 1;
  }

  /**
   * Checks whether the provided box and half-space are colliding. The two objects are colliding
   * whenever any of the box's eight vertices are penetrating or touching the half-space. If this is
   * the case, the method will generate either one or multiple new {@link Contact}s and add them to
   * the provided {@link CollisionData} object.
   *
   * @param box the box potentially involved in a collision
   * @param halfSpace the plane treated as a half-space potentially involved in a collision
   * @param data contact registry to which any found contacts will be added
   * @return the number of contacts generated
   */
  private static int boxAndHalfSpace(
      CollisionBox box, CollisionPlane halfSpace, CollisionData data) {
    // Ensure we actually have contact
    if (!boxAndHalfSpaceIntersect(box, halfSpace)) {
      return 0;
    }
    // Go through each combination of + and - for each half-size
    double[][] mults = {
      {1, 1, 1},
      {-1, 1, 1},
      {1, -1, 1},
      {-1, -1, 1},
      {1, 1, -1},
      {-1, 1, -1},
      {1, -1, -1},
      {-1, -1, -1}
    };
    int contactsUsed = 0;
    for (int i = 0; i < mults.length; i++) {
      // Calculate the position of each vertex
      Vector3D vertexPos = new Vector3D(mults[i][0], mults[i][1], mults[i][2]);
      vertexPos.componentProduct(box.getHalfSize());
      box.getTransform().transform(vertexPos);
      // Calculate the distance from the half-space
      double vertexDistance = vertexPos.scalarProduct(halfSpace.getDirection());
      // Compare this to the half-space's distance
      if (vertexDistance <= halfSpace.getOriginOffset()) {
        // Create contact data, the contact point is halfway between the vertex and the half-space
        Contact contact = new Contact();
        Vector3D contactPoint = new Vector3D(halfSpace.getDirection());
        contactPoint.multiply(vertexDistance - halfSpace.getOriginOffset());
        contactPoint.add(vertexPos);
        contact.setContactPoint(contactPoint);
        contact.setContactNormal(new Vector3D(halfSpace.getDirection()));
        contact.setPenetration(halfSpace.getOriginOffset() - vertexDistance);
        contact.setBodyData(box.getRigidBody(), null, data.getFriction(), data.getRestitution());
        data.add(contact);
        contactsUsed++;
      }
    }
    return contactsUsed;
  }

  /**
   * Checks whether the provided boxes are colliding. If this is the case, the method will generate
   * a {@link Contact} and add them to the provided {@link CollisionData} object.
   *
   * @param boxA the first box potentially involved in a collision
   * @param boxB the second box potentially involved in a collision
   * @param data contact registry to which any found contact will be added
   * @return the number of contacts generated, either zero or one
   */
  private static int boxAndBox(CollisionBox boxA, CollisionBox boxB, CollisionData data) {
    if (!boxAndBoxIntersect(boxA, boxB)) {
      return 0;
    }
    // Find the vector between the two centers
    Vector3D toCenter = Vector3D.subtract(boxB.getAxis(3), boxA.getAxis(3));
    // We start assuming there is no contact
    double pen = Double.MAX_VALUE;
    int best = 0xffffff;
    // Check each axis, returning if we find a separating axis and keeping track of the axis with
    // the smallest penetration otherwise
    OverlapCheck oc = new OverlapCheck();
    oc.checkOverlap(boxA, boxB, boxA.getAxis(0), toCenter, 0, pen);
    if (!oc.penetration) {
      return 0;
    } else {
      if (oc.smallestPenetration < pen) {
        pen = oc.smallestPenetration;
        best = oc.index;
      }
    }
    oc.checkOverlap(boxA, boxB, boxA.getAxis(1), toCenter, 1, pen);
    if (!oc.penetration) {
      return 0;
    } else {
      if (oc.smallestPenetration < pen) {
        pen = oc.smallestPenetration;
        best = oc.index;
      }
    }
    oc.checkOverlap(boxA, boxB, boxA.getAxis(2), toCenter, 2, pen);
    if (!oc.penetration) {
      return 0;
    } else {
      if (oc.smallestPenetration < pen) {
        pen = oc.smallestPenetration;
        best = oc.index;
      }
    }

    oc.checkOverlap(boxA, boxB, boxA.getAxis(0), toCenter, 3, pen);
    if (!oc.penetration) {
      return 0;
    } else {
      if (oc.smallestPenetration < pen) {
        pen = oc.smallestPenetration;
        best = oc.index;
      }
    }
    oc.checkOverlap(boxA, boxB, boxA.getAxis(1), toCenter, 4, pen);
    if (!oc.penetration) {
      return 0;
    } else {
      if (oc.smallestPenetration < pen) {
        pen = oc.smallestPenetration;
        best = oc.index;
      }
    }
    oc.checkOverlap(boxA, boxB, boxA.getAxis(2), toCenter, 5, pen);
    if (!oc.penetration) {
      return 0;
    } else {
      if (oc.smallestPenetration < pen) {
        pen = oc.smallestPenetration;
        best = oc.index;
      }
    }

    // Store the best axis-major, in case we run into almost parallel edge collisions later
    int bestSingleAxis = best;

    oc.checkOverlap(
        boxA, boxB, Vector3D.vectorProduct(boxA.getAxis(0), boxB.getAxis(0)), toCenter, 6, pen);
    if (!oc.penetration) {
      return 0;
    } else {
      if (oc.smallestPenetration < pen) {
        pen = oc.smallestPenetration;
        best = oc.index;
      }
    }
    oc.checkOverlap(
        boxA, boxB, Vector3D.vectorProduct(boxA.getAxis(0), boxB.getAxis(1)), toCenter, 7, pen);
    if (!oc.penetration) {
      return 0;
    } else {
      if (oc.smallestPenetration < pen) {
        pen = oc.smallestPenetration;
        best = oc.index;
      }
    }
    oc.checkOverlap(
        boxA, boxB, Vector3D.vectorProduct(boxA.getAxis(0), boxB.getAxis(2)), toCenter, 8, pen);
    if (!oc.penetration) {
      return 0;
    } else {
      if (oc.smallestPenetration < pen) {
        pen = oc.smallestPenetration;
        best = oc.index;
      }
    }
    oc.checkOverlap(
        boxA, boxB, Vector3D.vectorProduct(boxA.getAxis(1), boxB.getAxis(0)), toCenter, 9, pen);
    if (!oc.penetration) {
      return 0;
    } else {
      if (oc.smallestPenetration < pen) {
        pen = oc.smallestPenetration;
        best = oc.index;
      }
    }
    oc.checkOverlap(
        boxA, boxB, Vector3D.vectorProduct(boxA.getAxis(1), boxB.getAxis(1)), toCenter, 10, pen);
    if (!oc.penetration) {
      return 0;
    } else {
      if (oc.smallestPenetration < pen) {
        pen = oc.smallestPenetration;
        best = oc.index;
      }
    }
    oc.checkOverlap(
        boxA, boxB, Vector3D.vectorProduct(boxA.getAxis(1), boxB.getAxis(2)), toCenter, 11, pen);
    if (!oc.penetration) {
      return 0;
    } else {
      if (oc.smallestPenetration < pen) {
        pen = oc.smallestPenetration;
        best = oc.index;
      }
    }
    oc.checkOverlap(
        boxA, boxB, Vector3D.vectorProduct(boxA.getAxis(2), boxB.getAxis(0)), toCenter, 12, pen);
    if (!oc.penetration) {
      return 0;
    } else {
      if (oc.smallestPenetration < pen) {
        pen = oc.smallestPenetration;
        best = oc.index;
      }
    }
    oc.checkOverlap(
        boxA, boxB, Vector3D.vectorProduct(boxA.getAxis(2), boxB.getAxis(1)), toCenter, 13, pen);
    if (!oc.penetration) {
      return 0;
    } else {
      if (oc.smallestPenetration < pen) {
        pen = oc.smallestPenetration;
        best = oc.index;
      }
    }
    oc.checkOverlap(
        boxA, boxB, Vector3D.vectorProduct(boxA.getAxis(2), boxB.getAxis(2)), toCenter, 14, pen);
    if (!oc.penetration) {
      return 0;
    } else {
      if (oc.smallestPenetration < pen) {
        pen = oc.smallestPenetration;
        best = oc.index;
      }
    }

    // Make sure we've got a result
    if (best != 0xffffff) {
      // We now know there's a collision and we know which of the axes gave the smallest
      // penetration. We now can deal with it in different ways depending on the case.
      if (best < 3) {
        // We've got a vertex of boxB on a face of boxA
        fillPointFaceBoxBox(boxA, boxB, toCenter, data, best, pen);
        return 1;
      } else if (best < 6) {
        // We've got a vertex of boxA on a face of boxB. We use the same algorithm as above, but
        // swap around a and b (and therefore also the vector between their centers).
        toCenter.multiply(-1);
        fillPointFaceBoxBox(boxA, boxB, toCenter, data, best - 3, pen);
        return 1;
      } else {
        // We've got an edge-edge contact. Find out which axes.
        best -= 6;
        int axisIndexA = best / 3;
        int axisIndexB = best % 3;
        Vector3D axisA = boxA.getAxis(axisIndexA);
        Vector3D axisB = boxB.getAxis(axisIndexB);
        Vector3D axis = Vector3D.vectorProduct(axisA, axisB);
        axis.normalize();
        // The axis should point from box a to box b
        if (axis.scalarProduct(toCenter) > 0) {
          axis.multiply(-1);
        }
        // We have the axes, but not the edges: each axis has 4 edges parallel to it, we need to
        // find which of the 4 for each object. We do that by finding the point in the center of the
        // edge. We know its component in the direction of the box's collision axis is zero (its a
        // mid-point) and we determine which of the extremes in each of the other axes is closer.
        Vector3D ptOnEdgeA = new Vector3D(boxA.getHalfSize());
        Vector3D ptOnEdgeB = new Vector3D(boxB.getHalfSize());
        for (int i = 0; i < 3; i++) {
          if (i == axisIndexA) {
            ptOnEdgeA.insertComponent(i, 0);
          } else if (boxA.getAxis(i).scalarProduct(axis) > 0) {
            ptOnEdgeA.invertComponent(i);
          }
          if (i == axisIndexB) {
            ptOnEdgeB.insertComponent(i, 0);
          } else if (boxB.getAxis(i).scalarProduct(axis) < 0) {
            ptOnEdgeB.invertComponent(i);
          }
        }
        // Move them into world coordinates (they are already oriented correctly, since they have
        // been derived from the axes).
        boxA.getTransform().transform(ptOnEdgeA);
        boxB.getTransform().transform(ptOnEdgeB);
        // So we have a point and a direction for the colliding edges. We need to find out point of
        // closest approach of the two line-segments.
        Vector3D vertex =
            contactPoint(
                ptOnEdgeA,
                axisA,
                boxA.getHalfSize().getComponent(axisIndexA),
                ptOnEdgeB,
                axisB,
                boxB.getHalfSize().getComponent(axisIndexB),
                bestSingleAxis > 2);
        // Fill the contact with data
        Contact contact = new Contact();
        contact.setPenetration(pen);
        contact.setContactNormal(axis);
        contact.setContactPoint(vertex);
        contact.setBodyData(
            boxA.getRigidBody(), boxB.getRigidBody(), data.getFriction(), data.getRestitution());
        data.add(contact);
        return 1;
      }
    } else {
      return 0;
    }
  }

  /**
   * Checks whether the provided box and point are colliding. If the provided objects are colliding,
   * this method will generate a new {@link Contact} and add it to the provided data object.
   *
   * @param box the box potentially involved in a collision
   * @param point the point potentially involved in a collision
   * @param data contact registry to which any found contact will be added
   * @return the number of contacts generated, either zero or one
   */
  private static int boxAndPoint(CollisionBox box, Vector3D point, CollisionData data) {
    // Transform the point into box coordinates
    Vector3D relPt = box.getTransform().transformInverse(point);
    Vector3D normal;
    // Check each axis, looking for the axis on which the penetration is least deep
    double minDepth = box.getHalfSize().getX() - Math.abs(relPt.getX());
    if (minDepth < 0) {
      return 0;
    }
    normal = Vector3D.multiply(box.getAxis(0), ((relPt.getX() < 0) ? -1 : 1));
    double depth = box.getHalfSize().getY() - Math.abs(relPt.getY());
    if (depth < 0) {
      return 0;
    } else if (depth < minDepth) {
      minDepth = depth;
      normal = Vector3D.multiply(box.getAxis(1), ((relPt.getY() < 0) ? -1 : 1));
    }
    depth = box.getHalfSize().getZ() - Math.abs(relPt.getZ());
    if (depth < 0) {
      return 0;
    } else if (depth < minDepth) {
      minDepth = depth;
      normal = Vector3D.multiply(box.getAxis(2), ((relPt.getZ() < 0) ? -1 : 1));
    }
    // Create the contact
    Contact contact = new Contact();
    contact.setContactNormal(normal);
    contact.setContactPoint(point);
    contact.setPenetration(minDepth);
    contact.setBodyData(box.getRigidBody(), null, data.getFriction(), data.getRestitution());
    data.add(contact);
    return 1;
  }

  /**
   * Checks whether the provided box and sphere are colliding. If the provided objects are
   * colliding, this method will generate a new {@link Contact} and add it to the provided data
   * object.
   *
   * @param box the box potentially involved in a collision
   * @param sphere the sphere potentially involved in a collision
   * @param data contact registry to which any found contact will be added
   * @return the number of contacts generated, either zero or one
   */
  private static int boxAndSphere(CollisionBox box, CollisionSphere sphere, CollisionData data) {
    // Transform the center of the sphere into box coordinates
    Vector3D center = sphere.getAxis(3);
    Vector3D relCenter = box.getTransform().transformInverse(center);
    // Early out check to see if we can exclude the contact
    if (Math.abs(relCenter.getX()) - sphere.getRadius() > box.getHalfSize().getX()
        || Math.abs(relCenter.getY()) - sphere.getRadius() > box.getHalfSize().getY()
        || Math.abs(relCenter.getZ()) - sphere.getRadius() > box.getHalfSize().getZ()) {
      return 0;
    }
    Vector3D closestPoint = new Vector3D();
    double distance;
    // Clamp each coordinate to the box
    distance = relCenter.getX();
    if (distance > box.getHalfSize().getX()) {
      distance = box.getHalfSize().getX();
    }
    if (distance < -box.getHalfSize().getX()) {
      distance = -box.getHalfSize().getX();
    }
    closestPoint.setX(distance);

    distance = relCenter.getY();
    if (distance > box.getHalfSize().getY()) {
      distance = box.getHalfSize().getY();
    }
    if (distance < -box.getHalfSize().getY()) {
      distance = -box.getHalfSize().getY();
    }
    closestPoint.setY(distance);

    distance = relCenter.getZ();
    if (distance > box.getHalfSize().getZ()) {
      distance = box.getHalfSize().getZ();
    }
    if (distance < -box.getHalfSize().getZ()) {
      distance = -box.getHalfSize().getZ();
    }
    closestPoint.setZ(distance);

    // Check if we are in contact
    distance = Vector3D.subtract(closestPoint, relCenter).squareMagnitude();
    if (distance > sphere.getRadius() * sphere.getRadius()) {
      return 0;
    }
    // Create contact
    box.getTransform().transform(closestPoint);
    Contact contact = new Contact();
    Vector3D normal = Vector3D.subtract(closestPoint, center);
    normal.normalize();
    contact.setContactNormal(normal);
    contact.setContactPoint(closestPoint);
    contact.setPenetration(sphere.getRadius() - Math.sqrt(distance));
    contact.setBodyData(
        box.getRigidBody(), sphere.getRigidBody(), data.getFriction(), data.getRestitution());
    data.add(contact);
    return 1;
  }

  /**
   * Checks whether the given boxes are intersecting and returns the result of that check.
   *
   * @param boxA the first box
   * @param boxB the second box
   * @return true if the two boxes are intersecting, false if not
   */
  private static boolean boxAndBoxIntersect(CollisionBox boxA, CollisionBox boxB) {
    // Find the vector between the two centers
    Vector3D toCenter = Vector3D.subtract(boxB.getAxis(3), boxA.getAxis(3));
    return (
    // Check on boxA's axes first
    overlapOnAxis(boxA, boxB, boxA.getAxis(0), toCenter)
        && overlapOnAxis(boxA, boxB, boxA.getAxis(1), toCenter)
        && overlapOnAxis(boxA, boxB, boxA.getAxis(2), toCenter)
        // Check on boxB's axes
        && overlapOnAxis(boxA, boxB, boxB.getAxis(0), toCenter)
        && overlapOnAxis(boxA, boxB, boxB.getAxis(1), toCenter)
        && overlapOnAxis(boxA, boxB, boxB.getAxis(2), toCenter)
        // Now on the cross products
        && overlapOnAxis(
            boxA, boxB, Vector3D.vectorProduct(boxA.getAxis(0), boxB.getAxis(0)), toCenter)
        && overlapOnAxis(
            boxA, boxB, Vector3D.vectorProduct(boxA.getAxis(0), boxB.getAxis(1)), toCenter)
        && overlapOnAxis(
            boxA, boxB, Vector3D.vectorProduct(boxA.getAxis(0), boxB.getAxis(2)), toCenter)
        && overlapOnAxis(
            boxA, boxB, Vector3D.vectorProduct(boxA.getAxis(1), boxB.getAxis(0)), toCenter)
        && overlapOnAxis(
            boxA, boxB, Vector3D.vectorProduct(boxA.getAxis(1), boxB.getAxis(1)), toCenter)
        && overlapOnAxis(
            boxA, boxB, Vector3D.vectorProduct(boxA.getAxis(1), boxB.getAxis(2)), toCenter)
        && overlapOnAxis(
            boxA, boxB, Vector3D.vectorProduct(boxA.getAxis(2), boxB.getAxis(0)), toCenter)
        && overlapOnAxis(
            boxA, boxB, Vector3D.vectorProduct(boxA.getAxis(2), boxB.getAxis(1)), toCenter)
        && overlapOnAxis(
            boxA, boxB, Vector3D.vectorProduct(boxA.getAxis(2), boxB.getAxis(2)), toCenter));
  }

  /**
   * Checks whether the given box and half-space are intersecting and returns the result of that
   * check.
   *
   * @param box the box
   * @param halfSpace the plane treated as a half-space
   * @return true if the two objects are intersecting, false if not
   */
  private static boolean boxAndHalfSpaceIntersect(CollisionBox box, CollisionPlane halfSpace) {
    // Work out the projected radius of the box onto the plane direction
    double projectedRadius = transformToAxis(box, halfSpace.getDirection());
    // Work out how far the box is from the origin
    double boxDistance = halfSpace.getDirection().scalarProduct(box.getAxis(3)) - projectedRadius;
    // Check for the intersection
    return boxDistance <= halfSpace.getOriginOffset();
  }

  /**
   * Checks whether the given sphere and half-space are intersecting and returns the result of that
   * check.
   *
   * @param sphere the sphere
   * @param halfSpace the plane treated as a half-space
   * @return true if the two objects are intersecting, false if not
   */
  private static boolean sphereAndHalfSpaceIntersect(
      CollisionSphere sphere, CollisionPlane halfSpace) {
    // Find the distance from the origin
    double ballDistance = halfSpace.getDirection().scalarProduct(sphere.getAxis(3));
    ballDistance -= sphere.getRadius();
    // Check for the intersection
    return ballDistance <= halfSpace.getOriginOffset();
  }

  /**
   * Checks whether the given spheres are intersecting and returns the result of that check.
   *
   * @param sphereA the first sphere
   * @param sphereB the second sphere
   * @return true if the two objects are intersecting, false if not
   */
  private static boolean sphereAndSphereIntersect(
      CollisionSphere sphereA, CollisionSphere sphereB) {
    // Find vector between spheres
    Vector3D midLine = Vector3D.subtract(sphereA.getAxis(3), sphereB.getAxis(3));
    // See if it is short enough for an intersect
    return midLine.squareMagnitude()
        < (sphereA.getRadius() + sphereB.getRadius()) * (sphereA.getRadius() + sphereB.getRadius());
  }

  /**
   * Projects the given 3D box onto the provided axis and returns the total distance the box
   * occupies on that axis.
   *
   * @param box the box to project onto the {@code axis}
   * @param axis the axis to project the {@code box} onto
   * @return the distance occupied by the box on the axis
   */
  private static double transformToAxis(CollisionBox box, Vector3D axis) {
    return box.getHalfSize().getX() * Math.abs(axis.scalarProduct(box.getAxis(0)))
        + box.getHalfSize().getY() * Math.abs(axis.scalarProduct(box.getAxis(1)))
        + box.getHalfSize().getZ() * Math.abs(axis.scalarProduct(box.getAxis(2)));
  }

  /**
   * Calculates the overlap of the two provided 3D boxes when projected onto the given axis. A
   * positive return value indicates that the two boxes actually overlap while a negative value
   * indicates separation.
   *
   * <p>This is an implementation of the separating axis test (SAT). The SAT says that if there is
   * any axis along which two objects are separated, these objects can not be in contact. Thus, if
   * calling this method for all edges of two bodies returns a negative number at least once, the
   * objects are in contact.
   *
   * @param boxA the first box
   * @param boxB the second box
   * @param axis the axis the boxes will be projected onto
   * @param toCenter the distance between the two box's centers
   * @return the penetration depth of the two boxes on the axis
   */
  private static double penetrationOnAxis(
      CollisionBox boxA, CollisionBox boxB, Vector3D axis, Vector3D toCenter) {
    // Project the boxes onto the same axis
    double projectA = transformToAxis(boxA, axis);
    double projectB = transformToAxis(boxB, axis);
    // Project their distance onto the axis
    double distance = Math.abs(toCenter.scalarProduct(axis));
    // Return the overlap (positive indicates overlap, negative separation)
    return projectA + projectB - distance;
  }

  /**
   * Projects the given 3D boxes onto the provided axis and returns whether they overlap on that
   * axis.
   *
   * @param boxA the first box
   * @param boxB the second box
   * @param axis the axis on which the two boxes might overlap
   * @param toCenter the distance between the centers of the two boxes
   * @return true if the two boxes overlap on the axis, false if they don't
   * @see #transformToAxis(CollisionBox, Vector3D)
   */
  private static boolean overlapOnAxis(
      CollisionBox boxA, CollisionBox boxB, Vector3D axis, Vector3D toCenter) {
    // Project the boxes onto the axis
    double projectA = transformToAxis(boxA, axis);
    double projectB = transformToAxis(boxB, axis);
    // Project the distance onto the axis
    double distance = Math.abs(toCenter.scalarProduct(axis));
    // Check for overlap
    return (distance < projectA + projectB);
  }

  /**
   * Method capable of performing the final resolution of a vertex-face contact involving a vertex
   * of {@code boxA} touching a face of {@code boxB}. The method will resolve the contact and add
   * the data to the provided contact registry.
   *
   * @param boxA the first box involved in the contact
   * @param boxB the second box involved in the contact
   * @param toCenter the distance between the centers of the two boxes
   * @param data contact registry to which any found contacts will be added
   * @param best the axis of the contact
   * @param pen the contact's penetration depth
   */
  private static void fillPointFaceBoxBox(
      CollisionBox boxA,
      CollisionBox boxB,
      Vector3D toCenter,
      CollisionData data,
      int best,
      double pen) {
    // We know which axis the collision is on, but wee need to work out which of the two faces on
    // this axis
    Vector3D normal = boxA.getAxis(best);
    if (boxA.getAxis(best).scalarProduct(toCenter) > 0) {
      normal.multiply(-1);
    }
    // Work out which vertex of the second box we're colliding with
    Vector3D vertex = new Vector3D(boxB.getHalfSize());
    if (boxB.getAxis(0).scalarProduct(normal) < 0) {
      vertex.setX(-vertex.getX());
    }
    if (boxB.getAxis(1).scalarProduct(normal) < 0) {
      vertex.setY(-vertex.getY());
    }
    if (boxB.getAxis(2).scalarProduct(normal) < 0) {
      vertex.setZ(-vertex.getZ());
    }
    // Create contact data
    Contact contact = new Contact();
    contact.setContactNormal(normal);
    contact.setPenetration(pen);
    boxB.getTransform().transform(vertex);
    contact.setContactPoint(vertex);
    contact.setBodyData(
        boxA.getRigidBody(), boxB.getRigidBody(), data.getFriction(), data.getRestitution());
    data.add(contact);
  }

  /**
   * Determines and returns the closest point of approach of two line-segments. This method should
   * be used to determine the contact point of an edge-edge collision.
   *
   * @param pA the point on the edge of the first object
   * @param dA the transform vector of the object's edge {@code pA} is located on
   * @param sizeA the length of {@code dA}
   * @param pB the point on the edge of the second object
   * @param dB the transform vector of the object's edge {@code pB} is located on
   * @param sizeB the length of {@code dB}
   * @param useA indicates whether to return {@code pA} or {@code pB} if the determined contact
   *     point is not located on either of the edges
   * @return the contact point of the two edges
   */
  private static Vector3D contactPoint(
      Vector3D pA,
      Vector3D dA,
      double sizeA,
      Vector3D pB,
      Vector3D dB,
      double sizeB,
      boolean useA) {
    Vector3D toSt, cA, cB;
    double dpStaA, dpStaB, dpAB, smA, smB;
    double denom, mua, mub;

    smA = dA.squareMagnitude();
    smB = dB.squareMagnitude();
    dpAB = dA.scalarProduct(dB);

    toSt = Vector3D.subtract(pA, pB);
    dpStaA = dA.scalarProduct(toSt);
    dpStaB = dB.scalarProduct(toSt);

    denom = smA * smB - dpAB * dpAB;

    // Zero denominator indicates parallel lines
    if (Math.abs(denom) < 0.0001) {
      return useA ? new Vector3D(pA) : new Vector3D(pB);
    }

    mua = (dpAB * dpStaB - smB * dpStaA) / denom;
    mub = (smA * dpStaB - dpAB * dpStaA) / denom;

    // If either of the edges has the nearest point out of bounds, then the edges aren't crossed, we
    // have an edge-face contact. Our point is on the edge, which we know from the useA parameter.
    if (mua > sizeA || mua < -sizeA || mub > sizeB || mub < -sizeB) {
      return useA ? new Vector3D(pA) : new Vector3D(pB);
    } else {
      Vector3D tmp = new Vector3D(dA);
      tmp.multiply(mua);
      cA = Vector3D.add(pA, tmp);
      tmp = new Vector3D(dB);
      tmp.multiply(mub);
      cB = Vector3D.add(pB, tmp);
      cA.multiply(0.5);
      cB.multiply(0.5);
      return Vector3D.add(cA, cB);
    }
  }

  /**
   * Helper class for performing overlap checks on two boxes involved in a contact and storing the
   * results found by this check.
   */
  private static class OverlapCheck {

    /** Indicates whether the most recent overlap check found a penetration. */
    private boolean penetration;
    /**
     * The smallest penetration depth of all the collisions found by the overlap checks of this
     * object.
     */
    private double smallestPenetration;
    /** the index of the transform axis of the first box involved in any found contacts. */
    private int index;

    /**
     * Initializes the object with default values. Use {@link #checkOverlap(CollisionBox,
     * CollisionBox, Vector3D, Vector3D, int, double)} to analyze a collision between two boxes.
     */
    private OverlapCheck() {
      penetration = true;
      smallestPenetration = 0;
      index = 0;
    }

    /**
     * Checks whether the two given boxes collide on the provided axis and stores its findings in
     * this object.
     *
     * @param boxA the first box
     * @param boxB the second box
     * @param axis the axis the boxes might be colliding on
     * @param toCenter the distance between the two box's centers
     * @param index the index of the transform axis of the first box
     * @param smallestPenetration the smallest penetration depth out of all the depths of any
     *     contacts found between the two given boxes
     * @see #tryAxis(CollisionBox, CollisionBox, Vector3D, Vector3D, int, double)
     */
    private void checkOverlap(
        CollisionBox boxA,
        CollisionBox boxB,
        Vector3D axis,
        Vector3D toCenter,
        int index,
        double smallestPenetration) {
      double[] overlapResult = tryAxis(boxA, boxB, axis, toCenter, index, smallestPenetration);
      if (overlapResult[0] == 0) {
        penetration = false;
      } else {
        penetration = true;
        if (overlapResult[1] != 0.0) {
          this.smallestPenetration = overlapResult[1];
          this.index = (int) overlapResult[2];
        }
      }
    }

    /**
     * Checks whether the two given boxes collide on the provided axis and returns its findings.
     *
     * <p>The returned array will always contain three values. The first value indicates whether the
     * two boxes are actually penetrating each other (0 for no penetration, 1 for penetration). The
     * second value is the smallest penetration depth, which is either the given value or a new
     * value. The third and final value is the index of the corresponding vector in the first
     * provided box's transform matrix.
     *
     * @param boxA the first box
     * @param boxB the second box
     * @param axis the axis the boxes might be colliding on
     * @param toCenter the distance between the two box's centers
     * @param index the index of the transform axis of the first box
     * @param smallestPenetration the smallest penetration depth out of all the depths of any
     *     contacts found between the two given boxes
     * @return an array containing the findings of the method
     * @see #penetrationOnAxis(CollisionBox, CollisionBox, Vector3D, Vector3D)
     */
    private static double[] tryAxis(
        CollisionBox boxA,
        CollisionBox boxB,
        Vector3D axis,
        Vector3D toCenter,
        int index,
        double smallestPenetration) {
      double[] results = new double[3];
      // Make sure we have a normalized axis and don't check almost parallel axes
      if (axis.squareMagnitude() < 0.0001) {
        results[0] = 1;
        return results;
      }
      axis.normalize();
      double penetration = penetrationOnAxis(boxA, boxB, axis, toCenter);
      if (penetration < 0) {
        results[0] = 0;
        return results;
      }
      if (penetration < smallestPenetration) {
        results[1] = penetration;
        results[2] = index;
      } else {
        results[1] = smallestPenetration;
        results[2] = index;
      }
      results[0] = 1;
      return results;
    }
  }
}
