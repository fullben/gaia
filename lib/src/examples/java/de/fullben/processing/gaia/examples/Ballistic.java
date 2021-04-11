package de.fullben.processing.gaia.examples;

import de.fullben.processing.gaia.particles.Particle;
import java.util.ArrayList;
import java.util.List;
import processing.core.PApplet;

/**
 * A example for ballistic {@link Particle}s. {@code Ballistic} is a Processing sketch in which a
 * user may fire four different types of projectiles and observe their behavior.
 *
 * @author Benedikt Full
 */
public class Ballistic extends Example {

  private final List<Projectile> projectiles;
  private boolean mouseDown;
  private boolean prevMouseDown;
  private ProjectileType projectileType;

  public Ballistic() {
    projectiles = new ArrayList<>();
    projectileType = ProjectileType.PISTOL_BULLET;
  }

  public static void main(String[] args) {
    PApplet.main("de.fullben.processing.gaia.examples.Ballistic");
  }

  protected void update() {
    updateMouseStatus();
    if (mouseDown) {
      fire();
    }
    for (Projectile p : projectiles) {
      p.update(getFrameDuration());
    }
  }

  protected void display() {
    prepareContext();
    for (Projectile p : projectiles) {
      p.draw();
    }
    drawUi();
  }

  private void drawUi() {
    hint(DISABLE_DEPTH_TEST);
    pushStyle();
    camera();
    noLights();
    fill(255);
    textSize(18);
    int activeProjectiles = 0;
    for (Projectile p : projectiles) {
      if (p.getType() != ProjectileType.UNUSED) {
        activeProjectiles++;
      }
    }
    text("Total projectiles fired: " + projectiles.size(), 25, 30);
    text("Active projectiles: " + activeProjectiles, 25, 55);
    text("Selected projectile type: " + projectileType.toString(), 25, 80);
    text("Frames per second: " + (int) frameRate, 25, 105);
    textSize(14);
    text("Press any mouse button to launch a projectile.", 25, 135);
    text("Use 1 through 4 to change the projectile type.", 25, 150);
    popStyle();
    hint(ENABLE_DEPTH_TEST);
  }

  @Override
  public void keyPressed() {
    if (key == '1') {
      projectileType = ProjectileType.PISTOL_BULLET;
    } else if (key == '2') {
      projectileType = ProjectileType.ARTILLERY_SHELL;
    } else if (key == '3') {
      projectileType = ProjectileType.FIREBALL;
    } else if (key == '4') {
      projectileType = ProjectileType.LASER_BEAM;
    }
  }

  private Projectile initProjectile(ProjectileType type) {
    Projectile projectile = new Projectile(type);
    projectile.setPosition(0, 1.5, 0);
    switch (type) {
      case PISTOL_BULLET:
        projectile.setMass(2);
        projectile.setVelocity(0, 0, 350);
        projectile.setAcceleration(0, 10, 0);
        projectile.setDamping(0.99);
        break;
      case ARTILLERY_SHELL:
        projectile.setMass(200);
        projectile.setVelocity(0, -300, 400);
        projectile.setAcceleration(0, 200, 0);
        projectile.setDamping(0.99);
        break;
      case FIREBALL:
        projectile.setMass(1);
        projectile.setVelocity(0, 0, 100);
        projectile.setAcceleration(0, -6, 0);
        projectile.setDamping(0.9);
        break;
      case LASER_BEAM:
        projectile.setMass(0.1);
        projectile.setVelocity(0, 0, 1000);
        projectile.setAcceleration(0, 0, 0);
        projectile.setDamping(0.99);
        break;
    }
    return projectile;
  }

  private void fire() {
    projectiles.add(initProjectile(projectileType));
  }

  private void updateMouseStatus() {
    if (prevMouseDown && mousePressed) {
      mouseDown = false;
    } else if (!prevMouseDown && mousePressed) {
      mouseDown = true;
      prevMouseDown = true;
    } else if (!mousePressed) {
      prevMouseDown = false;
    }
  }

  private void prepareContext() {
    background(0);
    lights();
    rotateY((float) .75 * PI);
    translate(-500, pixelHeight / 2, 400);
    // stroke(255);
    fill(200);
    beginShape();
    vertex(-100, 200, -2000);
    vertex(100, 200, -2000);
    vertex(100, 200, 4000);
    vertex(-100, 200, 4000);
    endShape();
  }

  public enum ProjectileType {
    PISTOL_BULLET,
    ARTILLERY_SHELL,
    FIREBALL,
    LASER_BEAM,
    UNUSED
  }

  /** Particle objects used for simulating the different projectile types. */
  private class Projectile extends Particle {
    private ProjectileType type;
    private double startTime;

    Projectile(ProjectileType type) {
      this.type = type;
      startTime = System.currentTimeMillis();
    }

    protected void draw() {
      pushMatrix();
      noStroke();
      translate(
          (float) getPosition().getX(), (float) getPosition().getY(), (float) getPosition().getZ());
      rotateY((float) .5 * PI);
      if (type == ProjectileType.PISTOL_BULLET) {
        fill(100);
        sphere(5);
      } else if (type == ProjectileType.ARTILLERY_SHELL) {
        fill(0, 255, 0);
        sphere(40);
      } else if (type == ProjectileType.FIREBALL) {
        fill(255, 255, 0);
        sphere(50);
      } else if (type == ProjectileType.LASER_BEAM) {
        fill(0, 0, 255);
        box(200, 5, 5);
      }
      popMatrix();
    }

    protected void update(double duration) {
      if (type != ProjectileType.UNUSED) {
        integrate(duration * .001f);
        if (getPosition().getY() > 200
            || getPosition().getZ() > 5000
            || startTime + 20000 < System.currentTimeMillis()) {
          type = ProjectileType.UNUSED;
        }
      }
    }

    protected double getStartTime() {
      return startTime;
    }

    protected ProjectileType getType() {
      return type;
    }
  }
}
