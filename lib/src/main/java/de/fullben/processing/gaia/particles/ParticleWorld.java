package de.fullben.processing.gaia.particles;

import de.fullben.processing.gaia.Configuration;
import de.fullben.processing.gaia.particles.contacts.ParticleContact;
import de.fullben.processing.gaia.particles.contacts.ParticleContactResolver;
import de.fullben.processing.gaia.particles.pcgens.ParticleContactGenerator;
import de.fullben.processing.gaia.particles.pfgens.ParticleForceGenerator;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.List;

/**
 * A {@code ParticleWorld} keeps track of a set of particles and provides the means to update them
 * all.
 *
 * @author Benedikt Full
 * @see Particle
 */
public class ParticleWorld {

  /** The {@code Particle} objects inhabiting the world. */
  private final List<Particle> particles;
  /** The contact generators of the world. */
  private final List<ParticleContactGenerator> contactGenerators;
  /** The forces of the world. */
  private final ParticleForceRegistry forceRegistry;
  /** The {@code ParticleWorld}'s contact resolver. */
  private final ParticleContactResolver contactResolver;
  /** The current contacts occurring in the world. */
  private final List<ParticleContact> contacts;
  /** The maximum number of contacts allowed. */
  private final int maxContacts;
  /**
   * True if the world should calculate the number of iterations to give the contact resolver at
   * each frame.
   */
  private final boolean calculateIterations;
  /** The nanosecond timestamp of the last time the world was updated. */
  private long lastUpdateNanos;

  /**
   * Constructs a new particle simulator that can handle up to {@code maxContacts} contacts per
   * frame.
   *
   * @param maxContacts the maximum number of contacts that can be handled per frame
   * @param iterations number of contact resolution iterations
   */
  public ParticleWorld(int maxContacts, int iterations) {
    particles = new ArrayList<>();
    contactGenerators = new ArrayList<>();
    forceRegistry = new ParticleForceRegistry();
    contactResolver = new ParticleContactResolver(iterations);
    contacts = new ArrayList<>(maxContacts);
    this.maxContacts = maxContacts;
    calculateIterations = (iterations < 1);
    lastUpdateNanos = System.nanoTime();
  }

  /**
   * Updates the physics of entities populating this world based on the duration which has elapsed
   * since the last time this method or {@link #update(double)} was called.
   *
   * <p>Note that this method has no effect if the provided duration exceeds the maximum frame
   * duration value specified in the global configuration (available at {@link
   * Configuration#current()}), as processing the physics for longer durations generally only
   * produces undesirable results.
   *
   * @see #update(double)
   */
  public void update() {
    long currentNanoTime = System.nanoTime();
    long timeElapsed = currentNanoTime - lastUpdateNanos;
    lastUpdateNanos = currentNanoTime;
    runPhysics(timeElapsed / 1_000_000_000.0);
  }

  /**
   * Updates the physics of entities populating this world based on the given duration.
   *
   * <p>Note that this method has no effect if the provided duration exceeds the maximum frame
   * duration value specified in the global configuration (available at {@link
   * Configuration#current()}), as processing the physics for longer durations generally only
   * produces undesirable results.
   *
   * @param durationMillis duration in milliseconds which has elapsed since the last time the world
   *     was updated
   * @see #update()
   */
  public void update(double durationMillis) {
    lastUpdateNanos = System.nanoTime();
    runPhysics(durationMillis / 1_000.0);
  }

  /**
   * Processes all the physics for the this {@code ParticleWorld}.
   *
   * <p>This method has no effect if the duration exceeds the value of {@link
   * Configuration#getMaxFrameDuration()}, as processing the physics for longer durations generally
   * only produces undesirable results.
   *
   * @param duration duration for which to process in seconds, must be greater than zero and may not
   *     exceed the safety limit
   * @see #resetWorld()
   */
  private void runPhysics(double duration) {
    // Is the duration ok?
    if (duration <= 0 || duration > Configuration.current().getMaxFrameDuration()) {
      System.err.println("Attempted to run physics with unsafe frame duration: " + duration + "s");
      return;
    }
    // Apply the force generators
    // forceRegistry.updateForces(duration);
    forceRegistry.updateForces(duration);
    // Integrate objects
    integrate(duration);
    // Generate contacts
    int usedContacts = generateContacts();
    // Process contacts
    if (usedContacts > 0) {
      if (calculateIterations) {
        contactResolver.setIterations(usedContacts * 2);
      }
      contactResolver.resolveContacts(contacts, duration);
    }
    resetWorld();
  }

  /**
   * Prepares the world for a simulation frame. This clears the force accumulators for all the
   * particles registered with the world. After calling this, the particles can have their forces
   * for a new frame added.
   */
  private void resetWorld() {
    for (Particle p : particles) {
      // Remove all forces
      p.clearAccumulator();
    }
  }

  /**
   * Calls each of the contact generators registered with the world to report their contacts.
   *
   * @return the number of generated contacts
   */
  private int generateContacts() {
    int limit = maxContacts;
    int contactIndex = 0;
    contacts.clear(); // Removing all contacts from previous iteration
    for (ParticleContactGenerator g : contactGenerators) {
      ParticleContact contact = new ParticleContact();
      // int used = g.addContact(contacts.get(contactIndex), limit);
      int used = g.addContact(contact, limit);
      if (used > 0) {
        contacts.add(contact); // Adding newly generated contact
      }
      limit -= used;
      contactIndex += used;
      if (limit <= 0) {
        break;
      }
    }
    return maxContacts - limit;
  }

  /**
   * Integrates all the particles in the world forward in time by the given duration.
   *
   * @param duration duration for which to integrate in seconds
   */
  private void integrate(double duration) {
    for (Particle p : particles) {
      p.integrate(duration);
    }
  }

  /**
   * Adds the provided contact generator (e.g. a rod or a cable) to the world. All {@link Particle}
   * objects involved or affected by the generator have to be added to the world separately, e.g.
   * using {@link #addParticles(Collection)}. Not adding them will prevent the {@code Particle}s
   * from being actually affected by the generator.
   *
   * @param contactGenerator a contact generator
   * @see #addContactGenerators(ParticleContactGenerator...)
   * @see #addContactGenerators(Collection)
   */
  public void addContactGenerator(ParticleContactGenerator contactGenerator) {
    contactGenerators.add(contactGenerator);
  }

  /**
   * Adds the provided contact generators (e.g. rods, cables) to the world. All {@link Particle}
   * objects involved or affected by these generators have to be added to the world separately, e.g.
   * using {@link #addParticles(Collection)}. Not adding them will prevent the {@code Particle}s
   * from being actually affected by the generators.
   *
   * @param contactGenerators the contact generators to be added
   * @see #addContactGenerator(ParticleContactGenerator)
   * @see #addContactGenerators(Collection)
   */
  public void addContactGenerators(ParticleContactGenerator... contactGenerators) {
    if (contactGenerators == null) {
      return;
    }
    this.contactGenerators.addAll(Arrays.asList(contactGenerators));
  }

  /**
   * Adds the provided contact generators (e.g. rods, cables) to the world. All {@link Particle}
   * objects involved or affected by these generators have to be added to the world separately, e.g.
   * using {@link #addParticles(Collection)}. Not adding them will prevent the {@code Particle}s
   * from being actually affected by the generators.
   *
   * @param contactGenerators a list of contact generators
   * @see #addContactGenerator(ParticleContactGenerator)
   * @see #addContactGenerators(ParticleContactGenerator...)
   */
  public void addContactGenerators(Collection<ParticleContactGenerator> contactGenerators) {
    this.contactGenerators.addAll(contactGenerators);
  }

  /**
   * Adds the provided {@code Particle} to the world.
   *
   * @param particle a particle
   * @see #addParticles(Particle...)
   * @see #addParticles(Collection)
   */
  public void addParticle(Particle particle) {
    particles.add(particle);
    forceRegistry.register(particle);
  }

  /**
   * Adds the provided {@code Particle}s to the world.
   *
   * @param particles a list of particles
   * @see #addParticle(Particle)
   * @see #addParticles(Collection)
   */
  public void addParticles(Particle... particles) {
    if (particles == null) {
      return;
    }
    this.particles.addAll(Arrays.asList(particles));
  }

  /**
   * Adds the provided {@code Particle}s to the world.
   *
   * @param particles a list of particles
   * @see #addParticle(Particle)
   * @see #addParticles(Particle...)
   */
  public void addParticles(Collection<Particle> particles) {
    this.particles.addAll(particles);
    for (Particle particle : particles) {
      forceRegistry.register(particle);
    }
  }

  /**
   * Adds the provided force to the world. This force will be applied to all particles inhabiting
   * the world, regardless of whether the particles are added to the world before or after the force
   * has been added.
   *
   * @param force a force affecting all particles of the world
   */
  public void addForce(ParticleForceGenerator force) {
    forceRegistry.register(force);
  }
}
