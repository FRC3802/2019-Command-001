/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.OI;

import edu.wpi.first.wpilibj.Joystick;

public class DriverOI {
  //Driver Controller
  //Xbox One Wired Controller
  public Joystick controller;

  /** OI()
  * 1) Initializes Joysticks and buttons
  */
  public DriverOI() {
    controller = new Joystick(0);
  }
}
