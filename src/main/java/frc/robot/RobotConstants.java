// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.util.PIDConstant;

/**
 * Magic constants for the robot
 */
public class RobotConstants {

  // joystick ports
  public static final int DRIVER_PORT = 0;
  public static final int SPECIAL_OPS_PORT = 1;

  // special ops motor ports
  public static final int INTAKE_PORT = 9;
  public static final int SHOOTER_LAUNCH_PORT = 10;
  public static final int SHOOTER_INDEXER_PORT = 11;
  public static final int CLIMBER_EXTENDER_PORT = 12;
  public static final int CLIMBER_ROTATOR_PORT = 13;

  // special ops switch ports
  public static final int SHOOTER_SWITCH_PORT = 1;
  public static final int CLIMBER_EXTENDER_SWITCH = 3;
  public static final int CLIMBER_ROTATOR_SWITCH = 4;

  // drive motor ports
  public static final int FRONT_LEFT_ANGLE_ID = 8;
  public static final int FRONT_LEFT_DRIVE_ID = 7;
  public static final int FRONT_RIGHT_ANGLE_ID = 6;
  public static final int FRONT_RIGHT_DRIVE_ID = 5;
  public static final int BACK_RIGHT_ANGLE_ID = 4;
  public static final int BACK_RIGHT_DRIVE_ID = 3;
  public static final int BACK_LEFT_ANGLE_ID = 2;
  public static final int BACK_LEFT_DRIVE_ID = 1;

  // drive PID constants
  // public static final PIDConstant ANGLE_PID_CONSTANTS = new PIDConstant(75.0, 1e-3, 1.0);
  public static final PIDConstant DEFAULT_ANGLE_PID = new PIDConstant(1.0, 1e-4, 1.0);

  // drive constants
  public static final double MAX_DRIVE_POWER = 0.3;
  public static final double MAX_ROTATION_POWER = 5;
  public static final double DRIVE_MODE_ROTATION_LIMIT = 3;
  public static final double MAGIC_ROTATE_ANGLE = 2.72;
  public static final double ROTATION_SECONDS = 3.0;
  public static final double DEFAULT_ROTATION_SPEED = 2.0;

  // camera constants
  public static final boolean USE_CAMERAS = true;
  public static final int FRONT_CAMERA_PORT = 0;
  public static final int BACK_CAMERA_PORT = 1;
}