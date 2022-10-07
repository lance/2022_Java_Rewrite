// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.function.DoubleSupplier;

/** An example command that uses an example subsystem. */
public class TeleopDrive extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Drivetrain m_subsystem;
  private final DoubleSupplier xSpeed;
  private final DoubleSupplier zRotation;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public TeleopDrive(Drivetrain subsystem, DoubleSupplier xAxis, DoubleSupplier zAxis) {
    m_subsystem = subsystem;
    xSpeed = xAxis;
    zRotation = zAxis;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_subsystem.m_drive.arcadeDrive(xSpeed.getAsDouble(), zRotation.getAsDouble());
  }


  public void end(){
    m_subsystem.m_drive.arcadeDrive(0,0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
