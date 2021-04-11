package de.fullben.processing.gaia.rigidbodies.contacts;

import de.fullben.processing.gaia.math.Vector3D;
import java.util.ArrayList;
import java.util.List;
import processing.core.PShape;

/**
 *
 *
 * <h3>General</h3>
 *
 * A polygon mesh is a collection of vertices, edges and faces that defines the shape of a
 * polyhedral object in 3D space.
 *
 * <p>{@code Mesh} objects are not true polygon meshes, as they only store collections of vertices
 * and edges, ignoring any face data. This is due to the fact that simple bounding and collision
 * geometry generation can be performed without any information about the object's faces.
 *
 * <h3>Restrictions</h3>
 *
 * <p>The {@code Mesh} class is capable of analyzing and translating the data stored in a {@code
 * PShape} object as long as that object honors the following conditions:
 *
 * <ul>
 *   <li>The {@code PShape} has its faces stored in child shapes (always the case when initializing
 *       a shape based on an obj file)
 *   <li>All child shapes contain three or less vertices (in which case they'd either represent a
 *       vertex or an edge)
 * </ul>
 *
 * @author Benedikt Full
 */
public class Mesh {

  /** The vertices of the mesh. */
  private final List<Vector3D> vertices;
  /** The edges of the mesh. */
  private final List<Edge> edges;

  /**
   * Constructs a new mesh, storing the vertex and edge data of the given shape object.
   *
   * <p><b>Note:</b> Avoid providing {@code PShape} objects that have not been initialized using
   * {@code loadShape(String)}, as these types of shapes are most likely not compatible with the
   * engine.
   *
   * @param shape the rendering geometry of a rigid body
   */
  public Mesh(PShape shape) {
    this();
    translatePShape(shape);
  }

  /** Constructs a new, empty mesh. */
  public Mesh() {
    vertices = new ArrayList<>();
    edges = new ArrayList<>();
  }

  /**
   * Adds the provided shape's vertices and edges to this mesh. This process will fail if one of the
   * shape's faces has more than three vertices.
   *
   * <p><b>Note:</b> Avoid providing {@code PShape} objects that have not been initialized using
   * {@code loadShape(String)}, as these types of shapes are most likely not compatible with the
   * engine.
   *
   * @param shape the rendering geometry of a rigid body
   */
  private void translatePShape(PShape shape) {
    int vertexCount;
    // We assume that there are no vertices in the actual shape
    for (PShape s : shape.getChildren()) {
      vertexCount = s.getVertexCount();
      verifyVertexCount(vertexCount);
      if (vertexCount == 1) {
        Vector3D vertexA = new Vector3D();
        vertexA.setValues(s.getVertexX(0), s.getVertexY(0), s.getVertexZ(0));
        // Add the single vertex to vertex list
        addVertex(vertexA);
      } else if (vertexCount == 2) {
        Vector3D vertexA = new Vector3D();
        vertexA.setValues(s.getVertexX(0), s.getVertexY(0), s.getVertexZ(0));
        Vector3D vertexB = new Vector3D();
        vertexB.setValues(s.getVertexX(1), s.getVertexY(1), s.getVertexZ(1));
        // Adding the two vertices
        addVertex(vertexA);
        addVertex(vertexB);
        // Add the single edge represented by the face
        addEdge(new Edge(vertexA, vertexB));
      } else if (vertexCount == 3) {
        Vector3D vertexA = new Vector3D();
        vertexA.setValues(s.getVertexX(0), s.getVertexY(0), s.getVertexZ(0));
        Vector3D vertexB = new Vector3D();
        vertexB.setValues(s.getVertexX(1), s.getVertexY(1), s.getVertexZ(1));
        Vector3D vertexC = new Vector3D();
        vertexC.setValues(s.getVertexX(2), s.getVertexY(2), s.getVertexZ(2));
        // Adding the three vertices
        addVertex(vertexA);
        addVertex(vertexB);
        addVertex(vertexC);
        // Add all three edges of the face
        addEdge(new Edge(vertexA, vertexB));
        addEdge(new Edge(vertexA, vertexC));
        addEdge(new Edge(vertexB, vertexC));
      }
    }
  }

  /**
   * Verifies that the given vertex count of a {@code PShape} is valid. The count is valid if one of
   * the following conditions is true:
   *
   * <ul>
   *   <li>Count is zero - The shape is empty
   *   <li>Count is one - The shape represents just one vertex
   *   <li>Count is two - The shape represents only one edge
   *   <li>Count is three - The shape represents a regular face
   * </ul>
   *
   * The method will throw an {@link IllegalArgumentException} if none of the above conditions are
   * found to be true.
   *
   * @param vertexCount the number of vertices stored in a {@code PShape}
   */
  private void verifyVertexCount(int vertexCount) {
    if (vertexCount < 0 || vertexCount > 3) {
      throw new IllegalArgumentException("A face may not have more than 3 vertices");
    }
  }

  /**
   * Adds the provided vertex to the mesh. The vertex will only be added if it has not been
   * associated with the mesh yet.
   *
   * @param vertex the vertex to be added
   */
  private void addVertex(Vector3D vertex) {
    for (Vector3D v : vertices) {
      if (v.equals(vertex)) {
        return;
      }
    }
    vertices.add(vertex);
  }

  /**
   * Adds the provided edge to the mesh. The edge will only be added if it or another edge object
   * representing the same edge are not associated with the mesh yet.
   *
   * @param edge the edge to be added
   */
  private void addEdge(Edge edge) {
    for (Edge e : edges) {
      if (e.equals(edge)) {
        return;
      }
    }
    edges.add(edge);
  }

  /**
   * Returns the vertices making up this mesh.
   *
   * @return the vertices list
   */
  public List<Vector3D> getVertices() {
    return vertices;
  }

  /**
   * Returns the total number of vertices making up this mesh.
   *
   * @return the vertex count
   */
  public int getVertexCount() {
    return vertices.size();
  }

  public List<Edge> getEdges() {
    return edges;
  }

  /**
   * Clears this mesh object and fills it with the geometrical data of the provided shape. This
   * method should be called whenever the rendering geometry represented by this mesh changes (e.g.
   * new dimensions or new faces).
   *
   * @param shape the rendering geometry of a rigid body
   */
  public void updateGeometry(PShape shape) {
    vertices.clear();
    edges.clear();
    translatePShape(shape);
  }
}
