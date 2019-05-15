/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

public class RobotMap {
  // Hardware
  public static final int primaryPCM = 0;
  // DriveTrain
  public static final int leftFrontDrive = 56; // Talon SRX
  public static final int leftRearDrive = 35; // Victor SPX
  public static final int rightFrontDrive = 55; // Talon SRX
  public static final int rightRearDrive = 36; // Victor SPX

  public static final int elevatorDrive = 57; // Talon SRX

  // Software
  // Drive
  public static final boolean rightDriveReverse = false;
  public static final boolean rightDriveReverseEncoder = false;
  public static final boolean leftDriveReverse = false;
  public static final boolean leftDriveReverseEncoder = false;
  public static final double driveNominalOutputForward = 0;
  public static final double driveNominalOutputReverse = 0;
  public static final double drivePeakOutputForward = 1;
  public static final double drivePeakOutputReverse = -1;
  public static final int driveCruiseVelocity = 20000;
  public static final int driveAcceleration = 8000;
  // Drive PID
  public static final double drivePgain = 0.2;
  public static final double driveIgain = 0.0;
  public static final double driveDgain = 0.0;
  public static final double driveFgain = 0.2;

  // PID constants
  public static final int SlotIdx = 0;
  public static final int PIDLoopIdx = 0;
  public static final int TimeoutMs = 30;
}
