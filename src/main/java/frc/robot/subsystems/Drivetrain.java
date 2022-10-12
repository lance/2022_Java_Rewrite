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

  //TODO devise strategy for determining max wheel speed
  private final double k_maxLeftSpeed = 10; // m/s
  private final double k_maxRightSpeed = 10; // m/s
  private final double k_maxForwardSpeed = 10; // m/s
  private final double k_maxAngularSpeed = Math.PI / 5; // rad/s


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

  //Class takes two values [-1 to 1] for the desired speed on each side of the robot and  calls driveWheelSpeeds with the calculated and conditioned values
  public void tankDrive(double leftSpeed, double rightSpeed){
    //Apply NonLinear mapping
    leftSpeed = NonLinear(leftSpeed);
    rightSpeed = NonLinear(rightSpeed);
    //Limit values to desired range
    leftSpeed = MathUtil.clamp(leftSpeed, -1, 1);
    rightSpeed = MathUtil.clamp(rightSpeed, -1, 1);

    driveWheelSpeeds(new DifferentialDriveWheelSpeeds(leftSpeed * k_maxLeftSpeed, rightSpeed * k_maxRightSpeed));
  }

  //Takes two values [-1 to 1] for linear speed and angular rotation and calls driveChassisSpeeds with the calculated and conditioned values
  public void arcadeDrive(double xSpeed, double zRotation){
    //Apply NonLinear mapping
    xSpeed = NonLinear(xSpeed);
    zRotation = NonLinear(zRotation);
    //Limit values to desired range
    xSpeed = MathUtil.clamp(xSpeed, -1.0, 1.0);
    zRotation = MathUtil.clamp(zRotation, -1.0, 1.0);

    driveChassisSpeeds(new ChassisSpeeds(xSpeed * k_maxForwardSpeed, 0, zRotation * k_maxAngularSpeed));
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