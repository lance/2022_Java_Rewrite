// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final int climberLeadId = 1;
    public static final int climberFollowId = 2;
    public static final int climberRotateId = 3;
    public static final int stage1TalonId = 5;
    public static final int stage2TalonId = 4;
    public static final int stage3LeadSparkId = 6;
    public static final int stage3FollowSparkId = 7;
    public static final int leftFalconLeadId = 13;
    public static final int leftFalconFollowId = 12;
    public static final int rightFalconLeadId = 11;
    public static final int rightFalconFollowId = 10;
    public static final int compressorId = 8;
    public static final int solenoidPort = 0;
    
    public static final boolean leftInvert = false;
    public static final boolean rightInvert = true;

    public static final double accelMod = .65; //Temp tuning enabled .7
    public static final double accelRamp = 8; //Temp tuning enabled 8
    public static final double turnMod = .3; //Temp tuning enabled .3
    public static final double turnRamp = 20; //Temp tuning enabled 20
    public static final double boost = 2; // 2
    public static final double k_accelDis = 60000;
    public static final double k_Vlow = 28;
    public static final double k_Vhigh = 40;

    public static final int gamePadPort = 0;
    public static final int accelerationAxis = 1;
    public static final int steeringAxis = 2;
}
