// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.filter.SlewRateLimiter;

import java.lang.Math;
import edu.wpi.first.math.MathUtil;

import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {
  //Initalize motor controllers
  private final WPI_TalonFX m_leftLead = new WPI_TalonFX(Constants.CanId.leftDriveLead);
  private final WPI_TalonFX m_leftFollow = new WPI_TalonFX(Constants.CanId.leftDriveFollow);
  private final WPI_TalonFX m_rightLead = new WPI_TalonFX(Constants.CanId.rightDriveLead);
  private final WPI_TalonFX m_rightFollow = new WPI_TalonFX(Constants.CanId.rightDriveFollow);

  //Create motor controller groups
  private final MotorControllerGroup m_left = new MotorControllerGroup(m_leftLead,m_leftFollow); 
  private final MotorControllerGroup m_right = new MotorControllerGroup(m_rightLead,m_rightFollow);

  //Create Drivetrain controllers and kinematics objects
  private SimpleMotorFeedforward m_lFeedforward = new SimpleMotorFeedforward(Constants.Drive.Feedforward.Left.kS, Constants.Drive.Feedforward.Left.kV, Constants.Drive.Feedforward.Left.kA);
  private SimpleMotorFeedforward m_rFeedforward = new SimpleMotorFeedforward(Constants.Drive.Feedforward.Right.kS, Constants.Drive.Feedforward.Right.kV, Constants.Drive.Feedforward.Right.kA);
  private DifferentialDriveKinematics m_driveKinematics = new DifferentialDriveKinematics(Constants.Drive.kTrackWidth);

  private SlewRateLimiter m_accelLimiter = new SlewRateLimiter(Constants.Drive.Rate.driverForwardAccel);
  private SlewRateLimiter m_rotateLimiter = new SlewRateLimiter(Constants.Drive.Rate.driverRotateAccel);


  //Constructor taking no arguments, all relevant values are defined in Constants.java
  public Drivetrain() {
    //Set one drivetrain pair to run reverse so both drive forward on the positive direction (Which group selected by Constants.Drive.kInvertDrive; left = False)
    m_left.setInverted(!Constants.Drive.kInvertDrive);
    m_right.setInverted(Constants.Drive.kInvertDrive);
  }

  //Enable or disable brake mode on the motors
  public void brakeMode(boolean mode){
    NeutralMode nMode = NeutralMode.Coast;
    if (mode) nMode = NeutralMode.Brake;

    m_leftLead.setNeutralMode(nMode);
    m_leftFollow.setNeutralMode(nMode);
    m_rightLead.setNeutralMode(nMode);
    m_leftFollow.setNeutralMode(nMode);
  }

  //Sets the SlewRateLimiter accelration limits on 
  public void setAccelLimits(double forward, double rotate){
    m_accelLimiter = new SlewRateLimiter(forward);
    m_rotateLimiter = new SlewRateLimiter(rotate);
  }

  public void setAccelLimits(double forward, double rotate, double forwardInit, double rotateInit){
    m_accelLimiter = new SlewRateLimiter(forward, forwardInit);
    m_rotateLimiter = new SlewRateLimiter(rotate, rotateInit);
  }

  public void resetForwardLimiter(double value){
    m_accelLimiter.reset(value);
  }

  public void resetRotateLimiter(double value){
    m_rotateLimiter.reset(value);
  }

  //Class takes two values [-1 to 1] for the desired speed on each side of the robot and  calls driveWheelSpeeds with the calculated and conditioned values
  public void tankDrive(double leftSpeed, double rightSpeed, boolean boost){
    //Limit values to desired range
    leftSpeed = MathUtil.clamp(leftSpeed, -1, 1);
    rightSpeed = MathUtil.clamp(rightSpeed, -1, 1);
    
    double leftDriveSpeed = leftSpeed * (boost ? Constants.Drive.Rate.maxForwardSpeed : Constants.Drive.Rate.driverSpeed);
    double rightDriveSpeed = rightSpeed * (boost ? Constants.Drive.Rate.maxForwardSpeed : Constants.Drive.Rate.driverSpeed);

    driveWheelSpeeds(new DifferentialDriveWheelSpeeds(leftDriveSpeed, rightDriveSpeed));
  }

  public void tankDrive(double leftSpeed, double rightSpeed){
    tankDrive(leftSpeed, rightSpeed, false);
  }

  //Takes two values [-1 to 1] for linear speed and angular rotation and calls driveChassisSpeeds with the calculated and conditioned values
  public void arcadeDrive(double xSpeed, double zRotation, boolean boost, boolean rateLimit){
    //Limit values to desired range
    xSpeed = MathUtil.clamp(xSpeed, -1.0, 1.0);
    zRotation = MathUtil.clamp(zRotation, -1.0, 1.0);

    //Calculate the linear and rotation speeds requested by the inputs using either the boost(max) range, or the driver range
    double linearSpeed = xSpeed * (boost ? Constants.Drive.Rate.maxForwardSpeed : Constants.Drive.Rate.driverSpeed);
    double rotateSpeed = zRotation * Constants.Drive.Rate.driverRotate;

    if (rateLimit) driveChassisSpeeds(new ChassisSpeeds(m_accelLimiter.calculate(linearSpeed), 0, m_rotateLimiter.calculate(rotateSpeed)));
    else driveChassisSpeeds(new ChassisSpeeds(linearSpeed, 0, rotateSpeed));
  }

  //Arcade drive overloads for default parameters
  public void arcadeDrive(double xSpeed, double zRotation){
    arcadeDrive(xSpeed, zRotation, false, true);
  }

  public void arcadeDrive(double xSpeed, double zRotation, boolean boost){
    arcadeDrive(xSpeed, zRotation, boost, true);
  }

  //Set the appropriate motor voltages for a desired set of wheel speeds
  public void driveWheelSpeeds(DifferentialDriveWheelSpeeds wheelSpeeds){
    m_left.setVoltage(m_lFeedforward.calculate(wheelSpeeds.leftMetersPerSecond));
    m_right.setVoltage(m_rFeedforward.calculate(wheelSpeeds.rightMetersPerSecond));
  }

  //Set the appropriate motor voltages for a desired set of linear and angular chassis speeds
  public void driveChassisSpeeds(ChassisSpeeds chassisSpeeds){
    driveWheelSpeeds(m_driveKinematics.toWheelSpeeds(chassisSpeeds));
  }

  //Utility function to map joystick input nonlinearly for driver "feel"
  public static double NonLinear(double input){ return Math.copySign(input * input, input);}
}