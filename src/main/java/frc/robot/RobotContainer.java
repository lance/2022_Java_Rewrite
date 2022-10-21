// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;

import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.util.datalog.BooleanArrayLogEntry;

import java.util.function.BooleanSupplier;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public final XboxController m_driverController = new XboxController(Constants.primaryController);
  public final Joystick m_leftStick = new Joystick(1);
  public final Joystick m_rightStick = new Joystick(2);
  public final Drivetrain m_drivetrain = new Drivetrain();

  public final RunCommand m_teleopArcadeDrive =  new RunCommand(() -> m_drivetrain.arcadeDrive(
    Drivetrain.NonLinear(-m_driverController.getLeftY()),
    Drivetrain.NonLinear(m_driverController.getRightX()),
    m_driverController.getRightBumper()),
    m_drivetrain);

  public final RunCommand m_teleopTankDrive = new RunCommand(() -> m_drivetrain.tankDrive(
    Drivetrain.NonLinear(-m_leftStick.getY()),
    Drivetrain.NonLinear(-m_rightStick.getY()),
    m_rightStick.getTrigger()),
    m_drivetrain);

  public ShuffleboardTab driveTrainTab = Shuffleboard.getTab("Drivetrain");
  public NetworkTableEntry controlMode = driveTrainTab.add("Control Mode", true).withWidget(BuiltInWidgets.kToggleSwitch).getEntry();

  public final ConditionalCommand m_teleopDrive = new ConditionalCommand(m_teleopArcadeDrive, m_teleopTankDrive, () -> controlMode.getBoolean(true));

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    
  }
}
