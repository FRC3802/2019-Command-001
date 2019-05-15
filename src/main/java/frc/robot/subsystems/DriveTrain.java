/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import frc.robot.RobotMap;
import frc.robot.commandsDefault.DriveOpenLoop;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTableInstance;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

/**
 * An example subsystem.  You can replace me with your own Subsystem.
 */
public class DriveTrain extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private static DriveTrain Instance = null;
  private TalonSRX leftDrive;
  private TalonSRX rightDrive;
  private VictorSPX leftRearDriveSlave;
  private VictorSPX rightRearDriveSlave;

  public DriveTrain()
  {
    rightDrive = new TalonSRX(RobotMap.rightFrontDrive);
    leftDrive = new TalonSRX(RobotMap.leftFrontDrive);

    leftRearDriveSlave = new VictorSPX(RobotMap.leftRearDrive);
    rightRearDriveSlave = new VictorSPX(RobotMap.rightRearDrive);

    rightRearDriveSlave.follow(rightDrive);
    leftRearDriveSlave.follow(leftDrive);

    rightDrive.setNeutralMode(NeutralMode.Brake);
    leftDrive.setNeutralMode(NeutralMode.Brake);
    leftRearDriveSlave.setNeutralMode(NeutralMode.Brake);
    rightRearDriveSlave.setNeutralMode(NeutralMode.Brake);

    rightDrive.configFactoryDefault();
    leftDrive.configFactoryDefault();

    // the following is the original line, since we currently do not have CTRE, changed to QuadEncoder.  Might implement CTRE
    // Bud - 2019-05-04
    //rightDrive.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, RobotMap.PIDLoopIdx,RobotMap.TimeoutMs);
    rightDrive.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, RobotMap.PIDLoopIdx, RobotMap.TimeoutMs);
    // the following is the original line, since we currently do not have CTRE, changed to QuadEncoder.  Might implement CTRE
    // Bud - 2019-05-04
    //leftDrive.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, RobotMap.PIDLoopIdx,RobotMap.TimeoutMs);
    leftDrive.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, RobotMap.PIDLoopIdx,RobotMap.TimeoutMs);

    rightDrive.setSensorPhase(RobotMap.rightDriveReverse);
    rightDrive.setInverted(RobotMap.rightDriveReverseEncoder);
    leftDrive.setSensorPhase(RobotMap.leftDriveReverse);
    leftDrive.setInverted(RobotMap.leftDriveReverseEncoder);

    rightDrive.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, RobotMap.TimeoutMs);
    rightDrive.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, RobotMap.TimeoutMs);
    leftDrive.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, RobotMap.TimeoutMs);
    leftDrive.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, RobotMap.TimeoutMs);

    rightDrive.configNominalOutputForward(RobotMap.driveNominalOutputForward, RobotMap.TimeoutMs);
    rightDrive.configNominalOutputReverse(RobotMap.driveNominalOutputReverse, RobotMap.TimeoutMs);
    rightDrive.configPeakOutputForward(RobotMap.drivePeakOutputForward, RobotMap.TimeoutMs);
    rightDrive.configPeakOutputReverse(RobotMap.drivePeakOutputReverse, RobotMap.TimeoutMs);
    leftDrive.configNominalOutputForward(RobotMap.driveNominalOutputForward, RobotMap.TimeoutMs);
    leftDrive.configNominalOutputReverse(RobotMap.driveNominalOutputReverse, RobotMap.TimeoutMs);
    leftDrive.configPeakOutputForward(RobotMap.drivePeakOutputForward, RobotMap.TimeoutMs);
    leftDrive.configPeakOutputReverse(RobotMap.drivePeakOutputReverse, RobotMap.TimeoutMs);

    rightDrive.selectProfileSlot(RobotMap.SlotIdx, RobotMap.PIDLoopIdx);
    rightDrive.config_kF(RobotMap.SlotIdx, RobotMap.driveFgain, RobotMap.TimeoutMs);
    rightDrive.config_kP(RobotMap.SlotIdx, RobotMap.drivePgain, RobotMap.TimeoutMs);
    rightDrive.config_kI(RobotMap.SlotIdx, RobotMap.driveIgain, RobotMap.TimeoutMs);
    rightDrive.config_kD(RobotMap.SlotIdx, RobotMap.driveDgain, RobotMap.TimeoutMs);
    
    leftDrive.selectProfileSlot(RobotMap.SlotIdx, RobotMap.PIDLoopIdx);
    leftDrive.config_kF(RobotMap.SlotIdx, RobotMap.driveFgain, RobotMap.TimeoutMs);
    leftDrive.config_kP(RobotMap.SlotIdx, RobotMap.drivePgain, RobotMap.TimeoutMs);
    leftDrive.config_kI(RobotMap.SlotIdx, RobotMap.driveIgain, RobotMap.TimeoutMs);
    leftDrive.config_kD(RobotMap.SlotIdx, RobotMap.driveDgain, RobotMap.TimeoutMs);

    rightDrive.configMotionCruiseVelocity(RobotMap.driveCruiseVelocity, RobotMap.TimeoutMs);
    rightDrive.configMotionAcceleration(RobotMap.driveAcceleration, RobotMap.TimeoutMs);
    leftDrive.configMotionCruiseVelocity(RobotMap.driveCruiseVelocity, RobotMap.TimeoutMs);
    leftDrive.configMotionAcceleration(RobotMap.driveAcceleration, RobotMap.TimeoutMs);

    rightDrive.setSelectedSensorPosition(0);
    leftDrive.setSelectedSensorPosition(0);

  }

  public synchronized static DriveTrain getInstance() {
    if (Instance == null) {
        Instance = new DriveTrain();
    }
    return Instance;
}

  public void openLoop(double left, double right) {
    leftDrive.set(ControlMode.PercentOutput, left);
    rightDrive.set(ControlMode.PercentOutput, -right);
  }

  public void stop() {
    leftDrive.set(ControlMode.PercentOutput, 0);
    rightDrive.set(ControlMode.PercentOutput, 0);
  }

  public void postSmartDashVars(){
    SmartDashboard.putNumber("Left Drive ", leftDrive.getMotorOutputPercent());
    SmartDashboard.putNumber("Right Drive ", rightDrive.getMotorOutputPercent());
  }

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new DriveOpenLoop());
  }

  public double getRightCounts(){
    return rightDrive.getSelectedSensorPosition(RobotMap.PIDLoopIdx);
  }
  public double getLeftCounts(){
    return leftDrive.getSelectedSensorPosition(RobotMap.PIDLoopIdx);
  }
  public void setTrackingMode() {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3); //Turns LED on
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(0); //Begin Processing Vision
  }

  public void setDriverCamMode() {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1); //Turns LED off
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(1); //Disable Vision Processing and Doubles Exposure
  }
}
