package de.fullben.processing.gaia.examples;

import de.fullben.processing.gaia.math.Quaternion;
import de.fullben.processing.gaia.math.Vector3D;
import de.fullben.processing.gaia.rigidbodies.RigidBody;
import de.fullben.processing.gaia.rigidbodies.World;
import de.fullben.processing.gaia.rigidbodies.fgens.Gravity;
import java.text.DecimalFormat;
import java.util.ArrayList;
import java.util.List;
import processing.core.PApplet;
import processing.core.PConstants;

/**
 * An example for the rigid body engine. This is a Processing sketch in which the user may launch
 * spheres and observe their behavior. The bodies of the world are confined within a cuboid area,
 * which is encompassed by three visible and three invisible planar bounds.
 *
 * <p>Please note that the rigid body engine's capabilities are rather limited, which becomes very
 * obvious when either working with many rigid bodies or introducing shapes other than spheres.
 *
 * @author Benedikt Full
 */
public class RigidBallistic extends Example {

  private RigidBody cuboid;
  private RigidBody cuboidB;
  private World world;
  private Gravity gravity;
  private float timeScale = 1f;
  private int fpsMin;
  private int fpsMax;
  private boolean mouseDown;
  private boolean prevMouseDown;
  private List<RigidBody> shots;
  private int mode;
  private List<Hint> hints;
  private boolean prevKeyPressed;
  private boolean drawUi;
  private double gravityY;

  public RigidBallistic() {
    fpsMin = 0;
    fpsMax = 0;
    mode = 1;
    hints = new ArrayList<>();
    drawUi = true;
    gravityY = 10.0;
    initSpheres();
  }

  public static void main(String[] args) {
    PApplet.main("de.fullben.processing.gaia.examples.RigidBallistic");
  }

  /** Initializes the physics world without any cuboids for mode one. */
  private void initSpheres() {
    shots = new ArrayList<>();
    world = new World();
    gravity = new Gravity(0, gravityY, 0);
    world.addForce(gravity);
    addBounds();
  }

  /** Initializes the physics world with two cuboids for mode two. */
  private void initBallistic() {
    shots = new ArrayList<>();
    world = new World();
    gravity = new Gravity(0, gravityY, 0);
    // Cuboid
    double x = 2;
    double y = 2.5;
    double z = 2;
    cuboid = new RigidBody();
    cuboid.setPosition(0, 0.5, 0);
    cuboid.setMass((x / 2.0) * (y / 2.0) * (z / 2.0) * 5);
    cuboid.setInertiaTensorCuboid(x, y, z);
    // Second cuboid
    double xB = 1.5;
    double yB = 2;
    double zB = 1.5;
    cuboidB = new RigidBody();
    cuboidB.setPosition(0, -3, 0);
    cuboidB.setMass((xB / 2.0) * (yB / 2.0) * (zB / 2.0) * 2);
    cuboidB.setInertiaTensorCuboid(xB, yB, zB);
    world.addRigidBody(cuboid, x, y, z);
    world.addRigidBody(cuboidB, xB, yB, zB);
    world.addForce(gravity);
    addBounds();
  }

  /** Draws the 'gun' and projectiles created by the user. */
  private void drawShots() {
    // Gun
    pushMatrix();
    pushStyle();
    translate(0, -75, 550);
    noStroke();
    fill(55);
    box(75, 75, 150);
    popStyle();
    popMatrix();
    // Projectiles
    for (RigidBody b : shots) {
      pushMatrix();
      pushStyle();
      translate(b.getPosition().x(), b.getPosition().y(), b.getPosition().z());
      noStroke();
      fill(200, 250, 0);
      sphere(30);
      popStyle();
      popMatrix();
    }
  }

  /** Adds the immovable collision geometry representing the walls to the world. */
  private void addBounds() {
    world.addBound(new Vector3D(0, -1, 0), -2); // Floor
    world.addBound(new Vector3D(0, 1, 0), -8); // Ceiling
    world.addBound(new Vector3D(0, 0, 1), -5); // Left visible wall
    world.addBound(new Vector3D(1, 0, 0), -5); // Left invisible wall
    world.addBound(new Vector3D(-1, 0, 0), -5); // Right visible wall
    world.addBound(new Vector3D(0, 0, -1), -5); // Right invisible wall
  }

  @Override
  public void update() {
    world.update(getFrameDuration() * timeScale);
    updateFPSInfo();
    updateMouseStatus();
    fireProjectile();
    updateHints();
    updateKeyInput();
  }

  /**
   * Updates the {@link #prevKeyPressed} variable, which can be used to check whether a key has been
   * held during a previous update.
   */
  private void updateKeyStatus() {
    if (keyPressed) {
      if (!prevKeyPressed) {
        prevKeyPressed = true;
      }
    } else {
      prevKeyPressed = false;
    }
  }

  /** Updates the fields indicating whether or not the mouse has been pressed. */
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

  /** Calls all methods checking for user input made via keyboard. */
  private void updateKeyInput() {
    updateMode();
    updateTimeScale();
    toggleGravity();
    updateUIToggle();
    updateKeyStatus();
  }

  /** Checks for any mode toggles by the user and launches the appropriate mode. */
  private void updateMode() {
    if (keyPressed && !prevKeyPressed) {
      if (key == '1') {
        boolean reload = false;
        if (mode == 1) {
          reload = true;
        }
        mode = 1;
        initSpheres();
        cuboid = null;
        cuboidB = null;
        String hint;
        if (reload) {
          hint = "Reinitialized mode one";
        } else {
          hint = "Initialized mode one";
        }
        hints.add(new Hint(hint));
      } else if (key == '2') {
        boolean reload = false;
        if (mode == 2) {
          reload = true;
        }
        mode = 2;
        initBallistic();
        String hint;
        if (reload) {
          hint = "Reinitialized mode two";
        } else {
          hint = "Initialized mode two";
        }
        hints.add(new Hint(hint));
      }
    }
  }

  /** Updates the time scale based on the user input. */
  private void updateTimeScale() {
    double step = 0.1;
    if (keyPressed && !prevKeyPressed) {
      if (key == PConstants.CODED && keyCode == PConstants.DOWN) {
        if (timeScale > 0.1) {
          timeScale -= step;
          hints.add(new Hint("Slowed down time"));
        }
      } else if (key == PConstants.CODED && keyCode == PConstants.UP) {
        if (timeScale < 3.0) {
          timeScale += step;
          hints.add(new Hint("Sped up time"));
        }
      }
    }
  }

  /** Disables or enables the gravitational force of the world. */
  private void toggleGravity() {
    if (keyPressed && !prevKeyPressed) {
      if (key == 'g') {
        if (gravity.getGravity().getY() == 10.0) {
          gravityY = 0.0;
          hints.add(new Hint("Disabled gravity"));
        } else {
          gravityY = 10.0;
          hints.add(new Hint("Enabled gravity"));
        }
        gravity.getGravity().setY(gravityY);
      }
    }
  }

  /**
   * Sets the variable indicating whether the user wants to see the user interface or not according
   * to the input made via keyboard.
   */
  private void updateUIToggle() {
    if (keyPressed && !prevKeyPressed) {
      if (key == 't') {
        drawUi = !drawUi;
        if (drawUi) {
          hints.add(new Hint("Showing user interface"));
        } else {
          hints.add(new Hint("Hiding user interface"));
        }
      }
    }
  }

  /** Initializes a new projectile if a mouse button has been pressed. */
  private void fireProjectile() {
    if (mouseDown) {
      double radius = 0.3;
      RigidBody s = new RigidBody();
      s.setDamping(0.8, 0.8);
      s.setPosition(0, -0.75, 4.5);
      s.setVelocity(random(0.1f, 0.5f), random(0, 0.5f), -20);
      s.setMass((4.0 / 3.0) * PI * (radius * radius * radius) * 2);
      s.setInertiaTensorSphereSolid(radius);
      world.addRigidBody(s, radius);
      shots.add(s);
      hints.add(new Hint("New shot fired"));
    }
  }

  /** Updates the fields holding data related to frames per second / performance information. */
  private void updateFPSInfo() {
    if (millis() > 2000) {
      if (frameRate < fpsMin || fpsMin == 0) {
        fpsMin = (int) frameRate;
      }
      if (frameRate > fpsMax) {
        fpsMax = (int) frameRate;
      }
    }
  }

  @Override
  public void display() {
    prepareContext();
    drawArena();
    if (mode == 2) {
      drawTargets();
    }
    drawShots();
    if (drawUi) {
      drawUi();
    }
  }

  /**
   * Draws the current hints to the lower left corner of the screen. Hints describe the most recent
   * user induced actions.
   */
  private void drawHints() {
    int maxDisplayable = 6;
    float x = 25;
    float y = pixelHeight - 150;
    textSize(18);
    fill(200, 200, 100);
    noLights();
    camera();
    text("User Events", x, y);
    for (int i = 0; i < hints.size(); i++) {
      y += 20;
      if (i > maxDisplayable - 1) {
        break;
      }
      hints.get(i).draw(x + 2, y);
    }
  }

  /** Ages, fades and deletes the hints stored in the hints list. */
  private void updateHints() {
    float frameDuration = (float) getFrameDuration();
    int maxLife;
    int fadeDuration;
    if (hints.size() < 5) {
      maxLife = 800;
      fadeDuration = 400;
    } else {
      maxLife = 400;
      fadeDuration = 150;
    }
    if (hints.size() > 0) {
      // Focusing first hint, thus starting its life
      if (!hints.get(0).focused) {
        hints.get(0).setFocused();
      }
      // Initializing fading process if hint has reached max life duration
      if (!hints.get(0).fadeOut && hints.get(0).aliveFor > maxLife) {
        hints.get(0).fadeOut(fadeDuration);
      }
    }
    // Running the regular update routine of all hints
    for (Hint h : hints) {
      h.update(frameDuration);
    }
    // Removing faded hints
    ArrayList<Hint> newHints = new ArrayList<>();
    for (Hint h : hints) {
      if (!h.faded) {
        newHints.add(h);
      }
    }
    hints = newHints;
  }

  /** Initializes the lights and moves the camera to the appropriate position. */
  private void prepareContext() {
    background(0);
    // Lights
    lightSpecular(100, 100, 100);
    directionalLight(200, 200, 200, 0.5f, 1, -1);
    // Camera
    translate(650, pixelHeight / 1.5f, -650);
    rotateX(-PI / 10);
    rotateY(PI / 4.0f);
  }

  /** Draws the floor and two visible walls and adds grid cells to their surface. */
  private void drawArena() {
    int lineCount = 11;
    int offsetStep = 100;
    int gridColor = 50;
    pushMatrix();
    pushStyle();
    fill(200);
    beginShape();
    vertex(-500, 200, -500);
    vertex(500, 200, -500);
    vertex(500, 200, 500);
    vertex(-500, 200, 500);
    endShape();
    stroke(gridColor);
    int offset = 0;
    for (int i = 0; i < lineCount; i++) {
      line(-500 + offset, 200, -500, -500 + offset, 200, 500);
      line(-500, 200, -500 + offset, 500, 200, -500 + offset);
      offset += offsetStep;
    }
    noStroke();
    // Left wall
    beginShape();
    vertex(-500, 200, -500);
    vertex(-500, -800, -500);
    vertex(500, -800, -500);
    vertex(500, 200, -500);
    endShape();
    stroke(gridColor);
    offset = 0;
    for (int i = 0; i < lineCount; i++) {
      line(500, 200 - offset, -500, 500, 200 - offset, 500);
      line(500, 200, -500 + offset, 500, -800, -500 + offset);
      offset += offsetStep;
    }
    // Right wall
    beginShape();
    vertex(500, 200, 500);
    vertex(500, -800, 500);
    vertex(500, -800, -500);
    vertex(500, 200, -500);
    endShape();
    stroke(gridColor);
    offset = 0;
    for (int i = 0; i < lineCount; i++) {
      line(500, 200 - offset, -500, -500, 200 - offset, -500);
      line(-500 + offset, 200, -500, -500 + offset, -800, -500);
      offset += offsetStep;
    }
    popStyle();
    popMatrix();
  }

  /** Draws the target cuboids. */
  private void drawTargets() {
    drawCuboid();
    drawCuboidB();
  }

  /** Draws the first target cuboid. */
  private void drawCuboid() {
    pushMatrix();
    pushStyle();
    translate(cuboid.getPosition().x(), cuboid.getPosition().y(), cuboid.getPosition().z());
    Quaternion o = cuboid.getOrientation();
    rotateZ(o.z());
    rotateY(o.y());
    rotateX(o.x());
    noStroke();
    fill(255, 0, 255);
    box(200, 250, 200);
    popStyle();
    popMatrix();
  }

  /** Draws the second target cuboid. */
  private void drawCuboidB() {
    pushMatrix();
    pushStyle();
    translate(cuboidB.getPosition().x(), cuboidB.getPosition().y(), cuboidB.getPosition().z());
    Quaternion o = cuboidB.getOrientation();
    rotateZ(o.z());
    rotateY(o.y());
    rotateX(o.x());
    noStroke();
    fill(255, 0, 150);
    box(150, 200, 150);
    popStyle();
    popMatrix();
  }

  /**
   * Prepares the scene for drawing 2D text and calls all the methods responsible for drawing parts
   * of the user interface.
   *
   * @see #drawInfo()
   * @see #drawHints()
   * @see #drawDebug()
   */
  private void drawUi() {
    hint(DISABLE_DEPTH_TEST);
    pushMatrix();
    pushStyle();
    camera();
    noLights();
    drawInfo();
    drawHints();
    drawDebug();
    popStyle();
    popMatrix();
    hint(ENABLE_DEPTH_TEST);
  }

  /**
   * Draws information regarding performance, the current simulation and instructions for the user
   * to the left upper corner of the application window.
   */
  private void drawInfo() {
    fill(255);
    textSize(16);
    text("Current FPS: " + (int) frameRate, 25, 30);
    text("Minimum FPS: " + fpsMin, 25, 50);
    text("Maximum FPS: " + fpsMax, 25, 70);
    text("Rigid body count: " + world.getRigidBodyCount(), 25, 100);
    text("Contact count: " + world.getContactCount(), 25, 120);
    text("Gravitational acceleration: " + gravity.getGravity().getY(), 25, 150);
    text("Time scale: " + new DecimalFormat("#.#").format(timeScale), 25, 180);
    text(
        "Integration step (seconds): "
            + new DecimalFormat("#.###").format(getFrameDuration() * 0.001 * timeScale),
        25,
        200);
    textSize(14);
    text("Switch to or reset mode one: 1", 25, 230);
    text("Switch to or reset mode two: 2", 25, 250);
    text("Launch sphere: any mouse button", 25, 270);
    text("Accelerate or decelerate time: up/down arrow", 25, 290);
    text("Toggle gravity: g", 25, 310);
    text("Toggle user interface: t", 25, 330);
  }

  /**
   * Draws the position and orientation values of the {@link #cuboid} to the right upper corner of
   * the application window. The method will only draw this info whenever the cuboid is not null.
   */
  private void drawDebug() {
    if (cuboid != null) {
      float x = pixelWidth - 350;
      float col2 = 150;
      float initY = 50;
      float lineOffset = 25;
      fill(255, 0, 0);
      textSize(14);
      text("CUBOID (BLUE)", x + col2, initY + lineOffset);
      text("Position", x + col2, initY + 2 * lineOffset);
      text("X: " + cuboid.getPosition().x(), x + col2, initY + 3 * lineOffset);
      text("Y: " + cuboid.getPosition().y(), x + col2, initY + 4 * lineOffset);
      text("Z: " + cuboid.getPosition().z(), x + col2, initY + 5 * lineOffset);
      text("Orientation", x + col2, initY + 6 * lineOffset);
      text("X: " + cuboid.getOrientation().x(), x + col2, initY + 7 * lineOffset);
      text("Y: " + cuboid.getOrientation().y(), x + col2, initY + 8 * lineOffset);
      text("Z: " + cuboid.getOrientation().z(), x + col2, initY + 9 * lineOffset);
    }
  }

  /** Simple class for storing short messages which can be drawn to screen and fade out. */
  private class Hint {

    /** The hint's short, one-line text. */
    private final String text;
    /** Indicates whether this hint is the focused hint. A hint will only age if it is focused. */
    private boolean focused;
    /** The age of the hint in milliseconds. */
    private int aliveFor;
    /** The alpha value used to draw the hint. */
    private float alpha;
    /** Indicates whether the hint is currently fading (its alpha value is decreasing over time). */
    private boolean fadeOut;
    /** Indicates whether the hint's {@link #alpha} value has reached the minimum value. */
    private boolean faded;
    /**
     * The duration over which the alpha value is decreased from the maximum value to the minimum
     * value in milliseconds.
     */
    private int fadeDuration;

    /**
     * Initializes a hint with the given text.
     *
     * @param text the short one line message displayed by the hint
     */
    private Hint(String text) {
      this.text = text;
      focused = false;
      aliveFor = 0;
      alpha = 255;
      fadeOut = false;
      faded = false;
      fadeDuration = 0;
    }

    /**
     * Draws the hint's text at the given coordinates
     *
     * @param x the x coordinate
     * @param y the y coordinate
     */
    private void draw(float x, float y) {
      pushMatrix();
      pushStyle();
      camera();
      noLights();
      textSize(14);
      fill(200, 130, 100, alpha);
      text(text, x, y);
      popStyle();
      popMatrix();
    }

    /**
     * Sets the hint as focused. Calling a focused hint's {@link #update(float)} method will result
     * in the hint aging over time.
     */
    private void setFocused() {
      focused = true;
    }

    /**
     * Lowers the hint's alpha over the given duration from 255 to 100.
     *
     * @param duration time in milliseconds
     */
    private void fadeOut(int duration) {
      fadeOut = true;
      fadeDuration = duration;
    }

    /**
     * This method ages the hint by the given duration if it is focused. Additionally, this method
     * may decrease the hint's alpha value if the {@link #fadeOut} flag has been set.
     *
     * @param duration the duration of the most recent frame in milliseconds
     */
    private void update(float duration) {
      // Early out if hint has faded
      if (faded) {
        return;
      }
      // Aging hint if focused
      if (focused) {
        aliveFor += duration;
      }
      // Fading hint
      if (fadeOut) {
        alpha -= alpha * (duration / fadeDuration);
        if (alpha <= 100) {
          faded = true;
        }
      }
    }
  }
}
