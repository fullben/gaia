package de.fullben.processing.gaia.rigidbodies;

import de.fullben.processing.gaia.rigidbodies.fgens.ForceGenerator;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

/**
 * The {@code ForceRegistry} stores pairs of force generators and rigid bodies to which these forces
 * have to be applied to. The main purpose is to provide a convenient way for updating the forces of
 * multiple of rigid bodies. This can be achieved by utilizing the {@link #updateForces(double)}
 * method.
 *
 * @author Benedikt Full
 * @see ForceGenerator
 * @see RigidBody
 */
public class ForceRegistry {

  /** All the forces known to the registry. */
  private final Set<ForceGenerator> forces;
  /** All the bodies known to the registry. */
  private final Set<RigidBody> bodies;
  /**
   * A list of {@link ForceRegistration}s. Each of these objects contains a reference to one rigid
   * body and a force which acts on that body.
   */
  private final List<ForceRegistration> registrations;

  /** Constructs a new, empty force registry. */
  public ForceRegistry() {
    forces = new HashSet<>();
    bodies = new HashSet<>();
    registrations = new ArrayList<>();
  }

  /**
   * Registers the given force. This has the consequence that all {@link RigidBody rigid bodies}
   * currently known to the registry and added at some later point will be affected by this force.
   *
   * <p>Note that this method has no effect if the given force was already known to the registry.
   *
   * @param force the force to be added
   * @return {@code true} if the given force was not known to the registry and successfully added,
   *     {@code false} if the force was already known to the registry
   * @see #deregister(ForceGenerator)
   */
  public boolean register(ForceGenerator force) {
    if (!forces.add(force)) {
      return false;
    }
    for (RigidBody body : bodies) {
      registrations.add(new ForceRegistration(body, force));
    }
    return true;
  }

  /**
   * De-registers the given force. This means that after this method has been called, no {@link
   * RigidBody rigid bodies} will be affected by this force any longer.
   *
   * @param force the force to be removed from the registry
   * @return {@code true} if the force was registered and was successfully removed, {@code false} in
   *     any other case
   * @see #register(ForceGenerator)
   */
  public boolean deregister(ForceGenerator force) {
    if (!forces.remove(force)) {
      return false;
    }
    List<ForceRegistration> remove = new ArrayList<>();
    for (ForceRegistration registration : registrations) {
      if (registration.force.equals(force)) {
        remove.add(registration);
      }
    }
    return registrations.removeAll(remove);
  }

  /**
   * Registers the given rigid body. This has the consequence that the body will be affected by all
   * forces known to the registry, even if the forces are added after the body has already been
   * added.
   *
   * <p>Note that this method has no effect if the given rigid body was already known to the
   * registry.
   *
   * @param body the rigid body to be added
   * @return {@code true} if the body was not known to the registry and was successfully added,
   *     {@code false} if the body was already known to the registry
   * @see #deregister(RigidBody)
   */
  public boolean register(RigidBody body) {
    if (!bodies.add(body)) {
      return false;
    }
    for (ForceGenerator force : forces) {
      registrations.add(new ForceRegistration(body, force));
    }
    return true;
  }

  /**
   * De-registers the given rigid body. This means that after this method has been called the body
   * will no longer be affected by any forces.
   *
   * @param body the body to be removed from the registry
   * @return {@code true} if the body was registered and was successfully removed, {@code false} in
   *     any other case
   * @see #register(RigidBody)
   */
  public boolean deregister(RigidBody body) {
    if (!bodies.remove(body)) {
      return false;
    }
    List<ForceRegistration> remove = new ArrayList<>();
    for (ForceRegistration registration : registrations) {
      if (registration.body.equals(body)) {
        remove.add(registration);
      }
    }
    return registrations.removeAll(remove);
  }

  /** Removes all entries from the registry. */
  public void clear() {
    forces.clear();
    bodies.clear();
    registrations.clear();
  }

  /**
   * Calls all the registered force generators to update the forces of their corresponding bodies.
   *
   * @param duration the duration of the frame in seconds
   */
  public void updateForces(double duration) {
    for (ForceRegistration registration : registrations) {
      registration.updateForce(duration);
    }
  }

  /**
   * Pairs a rigid body with one of the force generators affecting that body.
   *
   * @author Benedikt Full
   */
  private static class ForceRegistration {

    /** A rigid body affected by the {@link #force}. */
    private final RigidBody body;
    /** A force affecting the {@link #body}. */
    private final ForceGenerator force;

    /**
     * Constructs a new registration linking the given body with the force generator.
     *
     * @param body a body affected by the given force generator
     * @param force a force affecting the provided body
     */
    private ForceRegistration(RigidBody body, ForceGenerator force) {
      this.body = body;
      this.force = force;
    }

    /**
     * Applies the force of {@link #force} to the {@link #body}.
     *
     * @param duration the duration for which to apply the force in seconds
     */
    private void updateForce(double duration) {
      force.updateForce(body, duration);
    }
  }
}
