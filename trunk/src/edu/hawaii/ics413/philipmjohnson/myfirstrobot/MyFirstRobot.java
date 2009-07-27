package edu.hawaii.ics413.philipmjohnson.myfirstrobot;

import robocode.HitByBulletEvent;
import robocode.Robot;
import robocode.ScannedRobotEvent;

/**
 * Illustrates basic robot behavior. 
 * Adapted from: http://robowiki.net/wiki/Robocode/My_First_Robot
 * Changes made: documentation, formatting. 
 * @author Philip Johnson
 */
public class MyFirstRobot extends Robot {

  /**
   * Implement a seesaw movement, performing a complete scan after each move.
   * Radar and gun always point in same direction. 
   */
  public void run() {
    while (true) {
      ahead(100); 
      turnGunRight(360); 
      back(100); 
      turnGunRight(360); 
    }
  }

  /**
   * Fire when we see a robot.
   * Targeting is simple since gun and radar point in the same direction.
   * @param e The ScannedRobotEvent, ignored since gun/radar now points at target. 
   */
  public void onScannedRobot(ScannedRobotEvent e) {
    fire(1);
  }

  /**
   * If hit, turn perpendicular to the bullet, so seesaw movement might avoid a future shot.
   * @param e The HitByBulletEvent instance.
   */
  public void onHitByBullet(HitByBulletEvent e) {
    turnLeft(90 - e.getBearing());
  }
}
