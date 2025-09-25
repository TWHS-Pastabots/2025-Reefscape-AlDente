//OLD CLAW CODE, DO NOT USE

package frc.robot.subsystems.claw;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Ports;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class Claw2 {
    public double ZeroPower;
    public SparkMax clawMotor;
    public SparkMaxConfig clawConfig;
    public static Claw instance;
    public Claw2(){
        clawMotor = new SparkMax(Ports.claw, MotorType.kBrushless);
        clawConfig = new SparkMaxConfig();

        clawConfig
            .inverted(true)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(60);
        clawMotor.configure(clawConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    }
    public void clawOn(double speed){
        clawMotor.set(speed);
    }
    public void clawReverse(double speed){
        clawMotor.set(-speed);
    }
    public void clawOff(double zeroPower){
        clawMotor.set(zeroPower);
    }
    public static Claw getInstance() {
        if (instance == null)
            instance = new Claw();
        return instance;
    }
}
