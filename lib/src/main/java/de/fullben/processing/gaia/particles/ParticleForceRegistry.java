package de.fullben.processing.gaia.particles;

import de.fullben.processing.gaia.particles.pfgens.ParticleForceGenerator;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

/**
 * The {@code ParticleForceRegistry} holds all the {@link ParticleForceGenerator}s and the {@link
 * Particle}s to which they have to be applied to. The main purpose is to provide a convenient way
 * for updating the forces of multiple of particles. This can be achieved by utilizing the {@link
 * #updateForces(double)} method.
 *
 * @author Benedikt Full
 */
public class ParticleForceRegistry {

  /** All the forces known to the registry. */
  private final Set<ParticleForceGenerator> forces;
  /** All the particles known to the registry. */
  private final Set<Particle> particles;
  /**
   * Holds a list of {@code ParticleForceRegistration}s. Each of these objects contains one particle
   * paired with a force applying to it.
   */
  private final List<ParticleForceRegistration> registrations;

  /** Constructs a new, empty particle-force-registry. */
  public ParticleForceRegistry() {
    forces = new HashSet<>();
    particles = new HashSet<>();
    registrations = new ArrayList<>();
  }

  /**
   * Registers the given force. This has the consequence that all {@link Particle}s currently known
   * to the registry and added at some later point will be affected by this force.
   *
   * <p>Note that this method has no effect if the given force was already known to the registry.
   *
   * @param force the force to be added
   * @return {@code true} if the given force was not known to the registry and successfully added,
   *     {@code false} if the force was already known to the registry
   * @see #deregister(ParticleForceGenerator)
   */
  public boolean register(ParticleForceGenerator force) {
    if (!forces.add(force)) {
      return false;
    }
    for (Particle particle : particles) {
      registrations.add(new ParticleForceRegistration(particle, force));
    }
    return true;
  }

  /**
   * De-registers the given force. This means that after this method has been called, no {@link
   * Particle}s will be affected by this force any longer.
   *
   * @param force the force to be removed from the registry
   * @return {@code true} if the force was registered and was successfully removed, {@code false} in
   *     any other case
   * @see #register(ParticleForceGenerator)
   */
  public boolean deregister(ParticleForceGenerator force) {
    if (!forces.remove(force)) {
      return false;
    }
    List<ParticleForceRegistration> remove = new ArrayList<>();
    for (ParticleForceRegistration registration : registrations) {
      if (registration.force.equals(force)) {
        remove.add(registration);
      }
    }
    return registrations.removeAll(remove);
  }

  /**
   * Registers the given particle. This has the consequence that the particle will be affected by
   * all forces known to the registry, even if the forces are added after the particle has been
   * added.
   *
   * <p>Note that this method has no effect if the given particle was already known to the registry.
   *
   * @param particle the particle to be added
   * @return {@code true} if the particle was not known to the registry and was successfully added,
   *     {@code false} if the particle was already known to the registry
   * @see #deregister(Particle)
   */
  public boolean register(Particle particle) {
    if (particles.contains(particle)) {
      return false;
    }
    particles.add(particle);
    for (ParticleForceGenerator force : forces) {
      registrations.add(new ParticleForceRegistration(particle, force));
    }
    return true;
  }

  /**
   * De-registers the given particle. This means that after this method has been called the particle
   * will no longer be affected by any forces.
   *
   * @param particle the particle to be removed from the registry
   * @return {@code true} if the particle was registered and was successfully removed, {@code false}
   *     in any other case
   * @see #register(ParticleForceGenerator)
   */
  public boolean deregister(Particle particle) {
    if (!particles.remove(particle)) {
      return false;
    }
    List<ParticleForceRegistration> remove = new ArrayList<>();
    for (ParticleForceRegistration registration : registrations) {
      if (registration.particle.equals(particle)) {
        remove.add(registration);
      }
    }
    return registrations.removeAll(remove);
  }

  /**
   * Removes all existing forces and particles from the registry. The registry will thus be empty
   * after this method has been executed.
   */
  public void clear() {
    forces.clear();
    particles.clear();
    registrations.clear();
  }

  /**
   * Calls all the registered force generators to update the forces of their corresponding
   * particles.
   *
   * @param duration the duration of the frame in seconds
   */
  public void updateForces(double duration) {
    for (ParticleForceRegistration registration : registrations) {
      registration.updateForce(duration);
    }
  }

  /**
   * Pairs a {@link Particle} with one of the {@link ParticleForceGenerator}s applying to it.
   *
   * @author Benedikt Full
   */
  private static class ParticleForceRegistration {

    /** A particle affected by the {@link #force}. */
    private final Particle particle;
    /** A force affecting the {@link #particle}. */
    private final ParticleForceGenerator force;

    /**
     * Constructs a new particle-force pair object, which can be used to associate {@link Particle}
     * objects with one of possibly multiple {@link ParticleForceGenerator} objects.
     *
     * @param particle a particle affected by the provided force generator
     * @param force a force affecting the provided particle
     */
    private ParticleForceRegistration(Particle particle, ParticleForceGenerator force) {
      this.particle = particle;
      this.force = force;
    }

    /**
     * Applies the force of {@link #force} to {@link #particle}.
     *
     * @param duration the duration of the frame in seconds
     */
    private void updateForce(double duration) {
      force.updateForce(particle, duration);
    }
  }
}
