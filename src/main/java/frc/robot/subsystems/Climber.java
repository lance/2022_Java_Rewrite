package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class Climber extends SubsystemBase {
    private final WPI_TalonSRX m_leadClimb = new WPI_TalonSRX(Constants.CanId.climberLiftLead);
    private final WPI_TalonSRX m_followClimb = new WPI_TalonSRX(Constants.CanId.climberLiftFollow);
    private final WPI_TalonSRX m_rotate = new WPI_TalonSRX(Constants.CanId.climberRotate);

    public Climber(){
        m_leadClimb.setInverted(Constants.motorConfigs[0]);
        m_followClimb.setInverted(Constants.motorConfigs[1]);
        m_rotate.setInverted(Constants.motorConfigs[2]);
        m_followClimb.follow(m_leadClimb);
        m_leadClimb.setNeutralMode(NeutralMode.Brake);
        m_followClimb.setNeutralMode(NeutralMode.Brake);
        m_rotate.setNeutralMode(NeutralMode.Brake);

    }

    void rotate(double rate){
        m_rotate.set(ControlMode.PercentOutput, rate);
    }
    
    void climb(double rate){
        m_leadClimb.set(ControlMode.PercentOutput, rate);
    }
    
    
    int getClimbLimit(){
        if(m_leadClimb.getSensorCollection().isFwdLimitSwitchClosed()){
            return 1;
        }
        else if(m_leadClimb.getSensorCollection().isRevLimitSwitchClosed()){
            return -1;
        }
        else return 0;
    }
    int getRotateLimit(){
        if(m_rotate.getSensorCollection().isFwdLimitSwitchClosed()){
            return 1;
        }
        else if(m_rotate.getSensorCollection().isRevLimitSwitchClosed()){
            return -1;
        }
        else return 0;
    }
}