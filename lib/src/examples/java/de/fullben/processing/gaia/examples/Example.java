package de.fullben.processing.gaia.examples;

import de.fullben.processing.gaia.math.Vector3D;
import processing.core.PApplet;

/**
 * An {@code Example} provides the initial setup for any demo of the physics engine's capabilities.
 * It initializes a Processing sketch running in a 1280 by 720 pixels window, utilizing Processing's
 * 3D renderer. Additionally this class stores the duration of the most recent frame, which may be
 * necessary for timing animations.
 *
 * <p>Subclasses should override {@link #update()} and {@link #display()}. These methods should
 * contain their respective update and drawing routines. For being able to run an example, it has to
 * have a main method which needs to contain the following method call: {@code
 * PApplet.main("package.Class");}
 *
 * @author Benedikt Full
 */
public abstract class Example extends PApplet {

  /** The point in time at which the previous frame ended. This is a milliseconds value. */
  private double prevFrameEnd;
  /** The duration of the most recent frame in milliseconds. */
  private double frameDuration;

  Example() {
    prevFrameEnd = System.currentTimeMillis();
    frameDuration = 1;
  }

  /**
   * Sets the application window size to 1280 by 720 pixels and sets the renderer to Processing's 3D
   * renderer.
   */
  @Override
  public void settings() {
    size(1280, 720, P3D);
    smooth(4);
  }

  /** Sets the frame rate - and thus update rate - to 200. */
  @Override
  public void setup() {
    frameRate(200);
  }

  /**
   * Calls {@link #update()} and {@link #display()} and saves the duration of the most recent frame,
   * which may be used for timing or quantifying animations.
   */
  @Override
  public void draw() {
    update();
    display();
    updateFrameDuration();
  }

  /**
   * This method should contain the example's update routines (e.g. recalculating positions of
   * objects or forces applying to them).
   */
  protected abstract void update();

  /**
   * This method should contain the example's draw routines (e.g. drawing a small cube at the
   * position of a particle).
   */
  protected abstract void display();

  /** Calculates and stores the duration of the most recent frame in milliseconds. */
  private void updateFrameDuration() {
    double currentTime = System.currentTimeMillis();
    frameDuration = currentTime - prevFrameEnd;
    prevFrameEnd = currentTime;
  }

  /**
   * Returns the duration of the previous frame.
   *
   * @return a milliseconds value
   */
  double getFrameDuration() {
    return frameDuration;
  }
}
