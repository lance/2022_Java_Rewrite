package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

public class Climber extends SubsystemBase {
    private final WPI_TalonFX m_leadClimb = new WPI_TalonFX(Constants.climberLeadId);
    private final WPI_TalonFX m_followClimb = new WPI_TalonFX(Constants.climberFollowId);
    private final WPI_TalonFX m_rotate = new WPI_TalonFX(Constants.climberRotateId);

    public Climber(){


    }

}