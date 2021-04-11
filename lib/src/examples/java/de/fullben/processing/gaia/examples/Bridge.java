package de.fullben.processing.gaia.examples;

import de.fullben.processing.gaia.math.Vector3D;
import de.fullben.processing.gaia.particles.Particle;
import de.fullben.processing.gaia.particles.ParticleWorld;
import de.fullben.processing.gaia.particles.pcgens.ParticleCable;
import de.fullben.processing.gaia.particles.pcgens.ParticleCableConstraint;
import de.fullben.processing.gaia.particles.pcgens.ParticleContactGenerator;
import de.fullben.processing.gaia.particles.pcgens.ParticleLink;
import de.fullben.processing.gaia.particles.pcgens.ParticleRod;
import de.fullben.processing.gaia.particles.pfgens.ParticleGravity;
import java.util.ArrayList;
import java.util.List;
import processing.core.PApplet;

/**
 * An example for the particle engine. This is a Processing sketch in which the user may move a
 * sphere along a rope suspended bridge.
 *
 * <p>At the point of the implementation of this example the engine is not yet capable of simulating
 * anything but particles and their constraints. Moving a sphere across a bridge can thus not be
 * simulated properly yet. In this example, this is achieved by employing a hack: The particles
 * closest to the sphere have their mass adjusted in such a way that it seems as if the sphere is
 * weighing down the bridge.
 *
 * @author Benedikt Full
 */
public class Bridge extends Example {

  private static final int ROD_COUNT = 6;
  private static final int CABLE_COUNT = 10;
  private static final int SUPPORT_COUNT = 12;
  private static final int BASE_MASS = 1;
  private static final int EXTRA_MASS = 10;
  private static final double STEP_LENGTH = 1.7;
  private List<Particle> particles;
  private List<ParticleContactGenerator> supports;
  private List<ParticleContactGenerator> cables;
  private List<ParticleContactGenerator> rods;
  private Vector3D massPos;
  private Vector3D massDisplayPos;
  private ParticleWorld world;

  public Bridge() {
    init();
  }

  public static void main(String[] args) {
    PApplet.main("de.fullben.processing.gaia.examples.Bridge");
  }

  /**
   * Creates the particle world and adds the particles and constraints defining the bridge to it.
   */
  private void init() {
    massPos = new Vector3D();
    massDisplayPos = new Vector3D();
    world = new ParticleWorld(500, 4);
    // Creating masses and connections?!
    particles = new ArrayList<>(12);
    for (int i = 0; i < 12; i++) {
      particles.add(i, new Particle());
    }
    int j = 0;
    for (Particle p : particles) {
      p.setPosition((j / 2) * 2.0 - 5.0, -4, (j % 2) * 2.0 - 1.0);
      p.setVelocity(0, 0, 0);
      p.setDamping(0.9);
      p.clearAccumulator();
      j++;
    }
    // Add the links
    cables = new ArrayList<>(CABLE_COUNT);
    for (int i = 0; i < CABLE_COUNT; i++) {
      cables.add(
          i, new ParticleCable(new Particle[] {particles.get(i), particles.get(i + 2)}, 1.9, 0.3));
    }
    world.addContactGenerators(cables);

    supports = new ArrayList<>(SUPPORT_COUNT);
    for (int i = 0; i < SUPPORT_COUNT; i++) {
      double maxLength;
      if (i < SUPPORT_COUNT / 2) {
        maxLength = (i / 2) * 0.5 + 3;
      } else {
        maxLength = (5.5) - (i / 2) * 0.5;
      }
      supports.add(
          i,
          new ParticleCableConstraint(
              particles.get(i),
              new Vector3D((i / 2) * 2.2 - 5.5, -6, (i % 2) * 1.6 - 0.8),
              maxLength,
              0.5));
    }
    world.addContactGenerators(supports);

    rods = new ArrayList<>(ROD_COUNT);
    for (int i = 0; i < ROD_COUNT; i++) {
      rods.add(
          i, new ParticleRod(new Particle[] {particles.get(i * 2), particles.get(i * 2 + 1)}, 2));
    }
    world.addContactGenerators(rods);

    world.addParticles(particles);
    world.addForce(new ParticleGravity());
    updateAdditionalMass();
  }

  /**
   * Increases the mass of the particles that are close to the player sphere, causing the bridge to
   * behave as if it was reacting to the player sphere moving on it.
   */
  private void updateAdditionalMass() {
    for (Particle p : particles) {
      p.setMass(BASE_MASS);
    }
    int x = (int) massPos.getX();
    double xp = massPos.getX() % 1.0;
    if (x < 0) {
      x = 0;
      xp = 0;
    }
    if (x >= 5) {
      x = 5;
      xp = 0;
    }

    int z = (int) massPos.getZ();
    double zp = massPos.getZ() % 1.0;
    if (z < 0) {
      z = 0;
      zp = 0;
    }
    if (z >= 1) {
      z = 1;
      zp = 0;
    }
    massDisplayPos.clear();
    particles.get(x * 2 + z).setMass(BASE_MASS + EXTRA_MASS * (1 - xp) * (1 - zp));
    massDisplayPos.addScaledVector(particles.get(x * 2 + z).getPosition(), (1 - xp) * (1 - zp));
    if (xp > 0) {
      particles.get(x * 2 + z + 2).setMass(BASE_MASS + EXTRA_MASS * xp * (1 - zp));
      massDisplayPos.addScaledVector(particles.get(x * 2 + z + 2).getPosition(), xp * (1 - zp));
      if (zp > 0) {
        particles.get(x * 2 + z + 3).setMass(BASE_MASS + EXTRA_MASS * xp * zp);
        massDisplayPos.addScaledVector(particles.get(x * 2 + z + 3).getPosition(), xp * zp);
      }
    }
    if (zp > 0) {
      particles.get(x * 2 + z + 1).setMass(BASE_MASS + EXTRA_MASS * (1 - xp) * zp);
      massDisplayPos.addScaledVector(particles.get(x * 2 + z + 1).getPosition(), (1 - xp) * zp);
    }
  }

  @Override
  protected void update() {
    movePlayer(getFrameDuration() * 0.001);
    updateAdditionalMass();
    world.update();
  }

  private void movePlayer(double duration) {
    double step = STEP_LENGTH * duration;
    if (keyPressed) {
      if (key == 'w') {
        massPos.setX(massPos.getX() + step);
        if (massPos.getX() > 5) {
          massPos.setX(5);
        }
      } else if (key == 's') {
        massPos.setX(massPos.getX() - step);
        if (massPos.getX() < 0) {
          massPos.setX(0);
        }
      } else if (key == 'a') {
        massPos.setZ(massPos.getZ() - step);
        if (massPos.getZ() < 0) {
          massPos.setZ(0);
        }
      } else if (key == 'd') {
        massPos.setZ(massPos.getZ() + step);
        if (massPos.getZ() > 1) {
          massPos.setZ(1);
        }
      }
    }
  }

  @Override
  protected void display() {
    // Scene setup
    prepareContext();
    // Particles
    for (Particle p : particles) {
      pushMatrix();
      noStroke();
      translate(p.getPosition().x(), p.getPosition().y(), p.getPosition().z());
      fill(0, 255, 0);
      sphere(10);
      popMatrix();
    }
    // Cables connecting the particles to each other
    for (int i = 0; i < cables.size(); i++) {
      if (cables.get(i) instanceof ParticleLink) {
        ParticleLink link = (ParticleLink) cables.get(i);
        Particle p1 = link.getParticleOne();
        Particle p2 = link.getParticleTwo();
        pushMatrix();
        stroke(255, 0, 0);
        line(
            p1.getPosition().x(),
            p1.getPosition().y(),
            p1.getPosition().z(),
            p2.getPosition().x(),
            p2.getPosition().y(),
            p2.getPosition().z());
        popMatrix();
      }
    }
    // Anchors and cables
    for (ParticleContactGenerator s : supports) {
      if (s instanceof ParticleCableConstraint) {
        ParticleCableConstraint cableConstraint = (ParticleCableConstraint) s;
        Vector3D anchor = cableConstraint.getAnchor();
        Particle p = cableConstraint.getParticle();
        pushMatrix();
        // Cables
        stroke(150);
        line(
            anchor.x(),
            anchor.y(),
            anchor.z(),
            p.getPosition().x(),
            p.getPosition().y(),
            p.getPosition().z());
        popMatrix();
        // Anchors
        pushMatrix();
        noStroke();
        translate(anchor.x(), anchor.y(), anchor.z());
        fill(0, 0, 255);
        box(15);
        popMatrix();
      }
    }
    // Rods connecting a pair of particles
    for (ParticleContactGenerator r : rods) {
      if (r instanceof ParticleRod) {
        ParticleRod rod = (ParticleRod) r;
        Particle p1 = rod.getParticleOne();
        Particle p2 = rod.getParticleTwo();
        pushMatrix();
        stroke(0, 255, 0);
        line(
            p1.getPosition().x(),
            p1.getPosition().y(),
            p1.getPosition().z(),
            p2.getPosition().x(),
            p2.getPosition().y(),
            p2.getPosition().z());
        popMatrix();
      }
    }
    // Player sphere
    drawPlayer();
    // User interface
    drawUi();
  }

  private void prepareContext() {
    background(0);
    lights();
    translate(pixelWidth / 2, 1.2f * pixelHeight, -700);
    rotateX(-.1f * PI);
    rotateY(0.15f * PI);
  }

  private void drawPlayer() {
    float radius = 30;
    pushStyle();
    translate(massDisplayPos.x(), massDisplayPos.y() - radius / 2, massDisplayPos.z());
    noStroke();
    fill(255, 255, 100);
    sphere(radius);
    popStyle();
  }

  private void drawUi() {
    hint(DISABLE_DEPTH_TEST);
    pushStyle();
    camera();
    noLights();
    fill(255);
    textSize(18);
    text("Frames per second: " + (int) frameRate, 25, 30);
    textSize(14);
    text("Use the WASD keys to move the sphere along the bridge.", 25, 60);
    popStyle();
    hint(ENABLE_DEPTH_TEST);
  }
}
