package de.fullben.processing.gaia.examples;

import de.fullben.processing.gaia.math.Vector3D;
import de.fullben.processing.gaia.particles.Particle;
import java.util.ArrayList;
import java.util.List;
import processing.core.PApplet;

/**
 * A example for {@link Particle} objects being used as fireworks. This is a Processing sketch in
 * which the user may fire as many fireworks as he pleases and observe their behavior.
 *
 * @author Benedikt Full
 */
public class Fireworks extends Example {

  private final List<Firework> fireworks;
  private List<FireworkRule> rules;
  private boolean mouseDown;
  private boolean prevMouseDown;

  public Fireworks() {
    fireworks = new ArrayList<>();
    initRules();
  }

  public static void main(String[] args) {
    PApplet.main("de.fullben.processing.gaia.examples.Fireworks");
  }

  protected void update() {
    updateMouseStatus();
    if (mouseDown) {
      fire();
    }
    for (Firework f : fireworks) {
      if (f.type > 0) {
        if (f.update(getFrameDuration())) {
          FireworkRule rule = rules.get(f.type - 1);
          f.type = 0;
          for (int i = 0; i < rule.payloadCount; i++) {
            f.children.add(rule.create((int) random(1, 4), f));
          }
        }
      }
      for (Firework c : f.children) {
        c.update(getFrameDuration());
      }
    }
  }

  protected void display() {
    prepareContext();
    for (Firework f : fireworks) {
      f.draw();
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
    int active = 0;
    for (Firework f : fireworks) {
      if (f.age > 0) {
        active++;
      }
      for (Firework c : f.children) {
        if (c.age > 0) {
          active++;
        }
      }
    }
    text("Total fireworks launched: " + fireworks.size(), 25, 30);
    text("Active fireworks/particles: " + active, 25, 55);
    text("Frames per second: " + (int) frameRate, 25, 80);
    textSize(14);
    text("Press any mouse button to launch a firework.", 25, 110);
    popStyle();
    hint(ENABLE_DEPTH_TEST);
  }

  /**
   * Initializes the {@link #rules} list, which contains {@link FireworkRule} object which define
   * the general behavior of a firework.
   */
  private void initRules() {
    rules = new ArrayList<>();
    FireworkRule rule1 =
        new FireworkRule(
            1, 0.5f, 1.4f, new Vector3D(-50, -250, -50), new Vector3D(50, -280, 50), 30);
    rule1.damping = 0.1;
    rules.add(rule1);
    FireworkRule rule2 =
        new FireworkRule(2, 0.5f, 1, new Vector3D(-50, -100, -50), new Vector3D(50, -200, 50), 60);
    rule2.damping = 0.1;
    rules.add(rule2);
    FireworkRule rule3 =
        new FireworkRule(
            3, 0.5f, 1.5f, new Vector3D(-100, -300, -50), new Vector3D(50, -350, 50), 40);
    rule3.damping = 0.1;
    rules.add(rule3);
  }

  /** Generates a {@link Firework} object based on a randomly selected rule. */
  private void fire() {
    int rule = (int) random(1, 4);
    fireworks.add(new Firework(rule, 2.5));
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

  /**
   * Sets up the world by setting the background color, lights and drawing a floor and fireworks
   * launcher.
   */
  private void prepareContext() {
    background(0);
    lights();
    rotateX(.1f * PI);
    translate(pixelWidth / 2f, .98f * pixelHeight, -400);
    noStroke();
    fill(0, 255, 0);
    box(50, 150, 50);
    fill(200);
    beginShape();
    vertex(-2500, 0, -1000);
    vertex(2500, 0, -1000);
    vertex(2500, 0, 2000);
    vertex(-2500, 0, 2000);
    endShape();
  }

  /**
   * {@code Firework} objects are {@link Particle}s capable of simulating an ascending firework
   * which will spawn additional fireworks upon exceeding their age limit.
   */
  private class Firework extends Particle {
    int type;
    double age;
    ArrayList<Firework> children;

    Firework(int type, double age) {
      this.type = type;
      this.age = age;
      children = new ArrayList<>();
      initValues();
    }

    private void initValues() {
      FireworkRule rule = rules.get(type - 1);
      setVelocity(Vector3D.random(rule.minVelocity, rule.maxVelocity));
    }

    boolean update(double duration) {
      duration = duration * 0.001f;
      integrate(duration);
      age -= duration;
      return (age < 0);
    }

    void draw() {
      if (age > 0) {
        pushMatrix();
        translate(
            (float) getPosition().getX(),
            (float) getPosition().getY(),
            (float) getPosition().getZ());
        fill(random(255), random(255), random(255));
        noStroke();
        box(random(8));
        popMatrix();
      }
      for (Firework c : children) {
        c.draw();
      }
    }
  }

  /**
   * {@code FireworkRule} objects can be used to define the general behavior of {@link Firework}
   * objects.
   */
  private class FireworkRule {
    private int type;
    private float minAge;
    private float maxAge;
    private Vector3D minVelocity;
    private Vector3D maxVelocity;
    private double damping;
    private int payloadCount;

    FireworkRule(
        int type,
        float minAge,
        float maxAge,
        Vector3D minVelocity,
        Vector3D maxVelocity,
        int payloadCount) {
      this.type = type;
      this.minAge = minAge;
      this.maxAge = maxAge;
      this.minVelocity = minVelocity;
      this.maxVelocity = maxVelocity;
      this.payloadCount = payloadCount;
      damping = 1;
    }

    Firework create(Firework parent) {
      Firework firework = new Firework(type, random(minAge, maxAge));
      Vector3D velocity = new Vector3D();
      if (parent != null) {
        firework.setPosition(
            parent.getPosition().getX(), parent.getPosition().getY(), parent.getPosition().getZ());
        firework.setVelocity(Vector3D.add(firework.getVelocity(), parent.getVelocity()));
      } else {
        Vector3D initialPos = new Vector3D();
        int y = (int) random(-500, 0);
      }
      velocity.add(Vector3D.random(minVelocity, maxVelocity));
      firework.setVelocity(velocity);
      firework.setMass(.5);
      firework.setDamping(damping);
      firework.setAcceleration(new Vector3D(0, 250, 0)); // gravity
      firework.clearAccumulator();
      return firework;
    }

    Firework create(int type, Firework parent) {
      FireworkRule rule = rules.get(type - 1);
      return rule.create(parent);
    }
  }
}
