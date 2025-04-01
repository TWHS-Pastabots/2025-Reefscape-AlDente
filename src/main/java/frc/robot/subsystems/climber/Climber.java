package frc.robot.subsystems.climber;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.wpilibj.Servo;
import frc.robot.Ports;
import frc.robot.subsystems.elevator.Elevator;


public class Climber {
    public SparkFlex climberMotor;
    public SparkFlexConfig MotorConfig;
    public static Climber instance;

    public Climber(){
        climberMotor = new SparkFlex(Ports.climber, MotorType.kBrushless);
    
        MotorConfig = new SparkFlexConfig();
        MotorConfig
            .inverted(false)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(60);
        climberMotor.configure(MotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void ClimbDown(){
        climberMotor.set(1);
    }
   

    
    public void ClimbUp(){
        climberMotor.set(-1);
    }

    public void ClimbZero(){
        climberMotor.set(0.0);
    }
     public static Climber getInstance() {
        if (instance == null)
            instance = new Climber();
        return instance;
    }
}
