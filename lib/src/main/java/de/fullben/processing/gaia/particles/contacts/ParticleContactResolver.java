package de.fullben.processing.gaia.particles.contacts;

import de.fullben.processing.gaia.math.Vector3D;
import java.util.List;

/**
 * Instances of this class are capable of resolving particle contacts.
 *
 * <p><b>Note:</b> The resolution of one contact may lead to the reactivation of a previously
 * resolved contact. This contact would then have to be resolved again. Putting a hard limit on the
 * number of possible re-resolutions of a contact prevents the algorithm from taking too long.
 *
 * @author Benedikt Full
 */
public class ParticleContactResolver {

  /** The number of resolution iterations allowed. */
  private int iterations;
  /** The number of iterations used during one resolution attempt. */
  private int iterationsUsed;

  /**
   * Constructs a new {@code ParticleContactResolver}. The number of resolution attempts when
   * resolving contacts is limited to the provided number.
   *
   * @param iterations number of resolution iterations allowed
   */
  public ParticleContactResolver(int iterations) {
    setIterations(iterations);
    iterationsUsed = 0;
  }

  /**
   * Sets the number of iterations that can be used when resolving contacts.
   *
   * @param iterations a non-negative number
   */
  public void setIterations(int iterations) {
    if (iterations < 0) {
      throw new IllegalArgumentException("The number of iterations may not be negative");
    } else {
      this.iterations = iterations;
    }
  }

  /**
   * Resolves a set of {@code Particle} contacts for both penetration and velocity.
   *
   * @param contacts the contacts to resolve
   * @param duration the duration for which to resolve in seconds
   */
  public void resolveContacts(List<ParticleContact> contacts, double duration) {
    int i;
    iterationsUsed = 0;
    while (iterationsUsed < iterations) {
      // Find contact with largest closing velocity
      double max = Double.MAX_VALUE;
      int maxIndex = contacts.size();
      for (i = 0; i < contacts.size(); i++) {
        double sepVel = contacts.get(i).calculateSeparatingVelocity();
        if (sepVel < max && (sepVel < 0 || contacts.get(i).getPenetration() > 0)) {
          max = sepVel;
          maxIndex = i;
        }
      }
      // Do we need to resolve anything?
      if (maxIndex == contacts.size()) {
        break;
      }
      // Resolve contact
      contacts.get(maxIndex).resolve(duration);
      // Update the interpenetrations for all particles
      Vector3D[] move = contacts.get(maxIndex).getParticleMovement();
      for (i = 0; i < contacts.size(); i++) {
        ParticleContact c = contacts.get(i);
        if (c.getParticleOne().equals(contacts.get(maxIndex).getParticleOne())) {
          c.setPenetration(c.getPenetration() - move[0].scalarProduct(c.getContactNormal()));
        } else if (c.getParticleOne().equals(contacts.get(maxIndex).getParticleTwo())) {
          c.setPenetration(c.getPenetration() - move[1].scalarProduct(c.getContactNormal()));
        }
        if (c.getParticleTwo() != null) {
          if (c.getParticleTwo().equals(contacts.get(maxIndex).getParticleOne())) {
            c.setPenetration(c.getPenetration() + move[0].scalarProduct(c.getContactNormal()));
          } else if (c.getParticleTwo().equals(contacts.get(maxIndex).getParticleTwo())) {
            c.setPenetration(c.getPenetration() + move[1].scalarProduct(c.getContactNormal()));
          }
        }
      }
      iterationsUsed++;
    }
  }
}
