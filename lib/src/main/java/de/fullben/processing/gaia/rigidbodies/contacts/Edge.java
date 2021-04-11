package de.fullben.processing.gaia.rigidbodies.contacts;

import de.fullben.processing.gaia.math.Vector3D;
import java.util.Objects;

/**
 * {@code Edge} objects represent an edge in 3D space. An edge is a connection between two vertices.
 *
 * @author Benedikt Full
 * @see Mesh
 */
class Edge {

  /** The first of the two vertices defining the edge. */
  private final Vector3D vertexA;
  /** The second of the two vertices defining the edge. */
  private final Vector3D vertexB;

  /**
   * Creates a new edge connecting the two provided vertices.
   *
   * @param vertexA the first vertex
   * @param vertexB the second vertex
   */
  Edge(Vector3D vertexA, Vector3D vertexB) {
    this.vertexA = vertexA;
    this.vertexB = vertexB;
  }

  /**
   * Returns the first of the edge's two vertices.
   *
   * @return {@link #vertexA}
   */
  Vector3D getVertexA() {
    return vertexA;
  }

  /**
   * Returns the second of the edge's two vertices.
   *
   * @return {@link #vertexB}
   */
  Vector3D getVertexB() {
    return vertexB;
  }

  /**
   * Checks whether this and the provided edge object represent the same edge in 3D space and
   * returns the result.
   *
   * @param o the edge to which this edge will be compared to
   * @return true if both objects represent the same edge
   */
  @Override
  public boolean equals(Object o) {
    if (this == o) {
      return true;
    }
    if (o == null || getClass() != o.getClass()) {
      return false;
    }
    Edge edge = (Edge) o;
    return Objects.equals(vertexA, edge.vertexA) && Objects.equals(vertexB, edge.vertexB);
  }

  @Override
  public int hashCode() {
    return Objects.hash(vertexA, vertexB);
  }
}
