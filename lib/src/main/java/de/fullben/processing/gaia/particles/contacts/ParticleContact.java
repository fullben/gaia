package de.fullben.processing.gaia.particles.contacts;

import de.fullben.processing.gaia.math.Vector3D;
import de.fullben.processing.gaia.particles.Particle;

/**
 * A {@code ParticleContact} represents two objects in contact. At least one of those objects has to
 * be a {@link Particle}. Resolving a contact removes their interpenetration and applies sufficient
 * impulse to keep them apart. Colliding bodies may also rebound.
 *
 * <p>The {@code ParticleContact} object has no callable methods, it simply holds the contact's
 * details. Objects of this class should be created whenever the collision detection algorithm of
 * this engine finds a pair of objects colliding.
 *
 * @author Benedikt Full
 */
public class ParticleContact {

  /**
   * The {@code Particle}s involved in the contact. Depending on the contact, this may either be two
   * particles or just one (e.g. when colliding with part of the world).
   */
  private final Particle[] particles;
  /**
   * Ratio of the final to initial relative velocity between two objects after they collide.
   */
  private double restitution;
  /**
   * Direction of the contact in world coordinates.
   */
  private Vector3D contactNormal;
  /**
   * Depth of the penetration at the contact.
   */
  private double penetration;
  /**
   * The amount each particle is moved by during interpenetration resolution.
   */
  private final Vector3D[] particleMovement;

  /**
   * Constructs a new, empty {@code ParticleContact}. Use the object's setter methods for adding
   * data.
   */
  public ParticleContact() {
    particles = new Particle[2];
    restitution = 0;
    contactNormal = new Vector3D();
    penetration = 0;
    particleMovement = new Vector3D[]{new Vector3D(), new Vector3D()};
  }

  /**
   * Resolves the contact for velocity and interpenetration.
   *
   * @param duration time interval over which to resolve in seconds
   */
  protected void resolve(double duration) {
    resolveVelocity(duration);
    resolveInterpenetration(duration);
  }

  /**
   * Calculates the separating velocity at the contact.
   *
   * @return the separating velocity
   */
  protected double calculateSeparatingVelocity() {
    Vector3D relativeVelocity = particles[0].getVelocity();
    if (particles[1] != null) {
      relativeVelocity.subtract(particles[1].getVelocity());
    }
    return relativeVelocity.scalarProduct(contactNormal);
  }

  /**
   * Performs the impulse calculation for this contact. This method has no effect if the contact is
   * separating or stationary.
   *
   * @param duration time interval over which to resolve in seconds
   */
  private void resolveVelocity(double duration) {
    // Find velocity in the direction of the contact
    double separatingVelocity = calculateSeparatingVelocity();
    // Check if it needs to be solved
    if (separatingVelocity > 0) {
      // Contact is separating or stationary
      return;
    }
    // Calculate new separating velocity
    double newSepVelocity = -separatingVelocity * restitution;
    // Check velocity buildup due to acceleration only
    Vector3D accCausedVelocity = particles[0].getAcceleration();
    if (particles[1] != null) {
      accCausedVelocity.subtract(particles[1].getAcceleration());
    }
    double accCausedSepVelocity = accCausedVelocity.scalarProduct(contactNormal) * duration;
    // If there is a closing velocity due to acceleration buildup, remove it from the separating vel
    if (accCausedSepVelocity < 0) {
      newSepVelocity += restitution * accCausedSepVelocity;
      if (newSepVelocity < 0) {
        newSepVelocity = 0;
      }
    }
    double deltaVelocity = newSepVelocity - separatingVelocity;
    // Apply change in velocity in proportion to inverse mass of objects
    // i.e. those with a lower inverse mass (higher actual mass) experience less change in velocity
    double totalInverseMass = particles[0].getInverseMass();
    if (particles[1] != null) {
      totalInverseMass += particles[1].getInverseMass();
    }
    // The particles have infinite mass, impulse has no effect
    if (totalInverseMass <= 0) {
      return;
    }
    // Calculate impulse to apply
    double impulse = deltaVelocity / totalInverseMass;
    Vector3D impulsePerInvMass = Vector3D.multiply(contactNormal, impulse);
    // Apply impulse in direction of contact and proportional to the inverse mass
    Vector3D velocity = particles[0].getVelocity();
    velocity.add(Vector3D.multiply(impulsePerInvMass, particles[0].getInverseMass()));
    particles[0].setVelocity(velocity);
    if (particles[1] != null) {
      velocity = particles[1].getVelocity();
      velocity.add(Vector3D.multiply(impulsePerInvMass, -particles[1].getInverseMass()));
      particles[1].setVelocity(velocity);
    }
  }

  /**
   * Performs the interpenetration resolution for this contact. Calling this method has no effect
   * until an actual interpenetration between the objects occurs.
   *
   * @param duration the duration for which to resolve the interpenetration in seconds
   */
  private void resolveInterpenetration(double duration) {
    // Don't do anything if there is no interpenetration
    if (penetration <= 0) {
      return;
    }
    // Movement of each object is based on their inverse mass, total that
    double totalInverseMass = particles[0].getInverseMass();
    if (particles[1] != null) {
      totalInverseMass += particles[1].getInverseMass();
    }
    // Don't do anything if objects have infinite mass
    if (totalInverseMass <= 0) {
      return;
    }
    // Find amount of penetration resolution per unit of inverse mass
    Vector3D movePerInvMass = Vector3D.multiply(contactNormal, (penetration / totalInverseMass));
    particleMovement[0] = Vector3D.multiply(movePerInvMass, particles[0].getInverseMass());
    if (particles[1] != null) {
      particleMovement[1] = Vector3D.multiply(movePerInvMass, -particles[1].getInverseMass());
    }
    // Apply penetration resolution
    Vector3D position = particles[0].getPosition();
    position.add(particleMovement[0]);
    particles[0].setPosition(position);
    if (particles[1] != null) {
      position = particles[1].getPosition();
      position.add(particleMovement[1]);
      particles[1].setPosition(position);
    } else {
      particleMovement[1].clear();
    }
  }

  /**
   * Returns the first of the possibly two particles involved in the contact.
   *
   * @return a {@code Particle}
   */
  public Particle getParticleOne() {
    return particles[0];
  }

  /**
   * Sets the provided {@code Particle} as the first of the possibly two particles involved in the
   * contact.
   *
   * @param particle a {@code Particle} involved in the contact
   */
  public void setParticleOne(Particle particle) {
    particles[0] = particle;
  }

  /**
   * Returns the second {@code Particle} involved in the contact. If the contact involves only one
   * {@code Particle}, this method may return null.
   *
   * @return a {code Particle} or null
   */
  public Particle getParticleTwo() {
    return particles[1];
  }

  /**
   * Sets the provided {@code Particle} as the second of the two particles involved in the contact.
   *
   * @param particle a {@code Particle} involved in the contact
   */
  public void setParticleTwo(Particle particle) {
    particles[1] = particle;
  }

  /**
   * Sets the provided value as the contact's {@link #restitution}.
   *
   * @param restitution the restitution coefficient of the collision
   */
  public void setRestitution(double restitution) {
    this.restitution = restitution;
  }

  /**
   * Returns the contact's normal vector.
   *
   * @return the direction of the contact
   */
  public Vector3D getContactNormal() {
    return contactNormal;
  }

  /**
   * Sets the provided value as the contact's {@link #contactNormal}.
   *
   * @param contactNormal direction of the contact
   */
  public void setContactNormal(Vector3D contactNormal) {
    this.contactNormal = contactNormal;
  }

  /**
   * Returns the depth of the interpenetration of the objects involved in the contact. If the value
   * is zero or negative, the objects are either separating or resting.
   *
   * @return the penetration depth
   */
  public double getPenetration() {
    return penetration;
  }

  /**
   * Sets the depth of the interpenetration of the contact to the provided value. If the value is
   * zero or negative, the objects are not penetrating each other and are thus either separating or
   * resting.
   *
   * @param penetration the penetration depth
   */
  public void setPenetration(double penetration) {
    this.penetration = penetration;
  }

  /**
   * Returns the amount each of the two particles is moved by during the interpenetration
   * resolution. The amount the first vector is moved is located at index {@code 0}, the amount for
   * the second vector at index {@code 1}.
   *
   * @return an array containing two vectors
   */
  public Vector3D[] getParticleMovement() {
    return particleMovement;
  }
}
