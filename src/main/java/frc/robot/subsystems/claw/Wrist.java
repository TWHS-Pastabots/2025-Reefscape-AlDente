//OLD WRIST CODE, DO NOT USE

package frc.robot.subsystems.claw;


import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Ports;
import frc.robot.subsystems.pivot.Pivot;

import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

public class Wrist  {
    public enum WristState {
        GROUND(0,115),
        LOWALGAEINTAKE(0,140),
        HIGHALGAEINTAKE(0,119),
        L1CORALSCORE(0,0), 
        L2CORALSCORE(270,167), 
        L3CORALSCORE(270,167),
        L4CORALSCORE(270,167),
        PROCESSOR(0,60),
        HUMANSTATIONINTAKE(0,74),
        TRANSITIONSTATE(0,90),
        TEST(0,110),
        CLIMB(0,84.6),
        
        NET(0,90);
        public double rotate;
        public double tilt;
        private WristState(double rotate, double tilt) {
            this.rotate = rotate;
            this.tilt = tilt;
            
        }
    }
    public double lVolts;
    public double rVolts;
    public SparkMax MotorR;
    public SparkMax MotorL;
    public SparkMaxConfig MotorConfigL;
    public SparkMaxConfig MotorConfigR;
    private static Wrist DW;
    public ArmFeedforward feedforwardL;
    public ArmFeedforward feedforwardR;
    
    // PID probably good enough
    public PIDController pitchPID = new PIDController(10, 0, 0);
    public PIDController rollPID = new PIDController(70, 0, 0);
    
    // // Motors
    // private SparkFlex leftMotor = new SparkFlex(WristIDs.kDiffWristLeftMotorID, MotorType.kBrushless);
    // private SparkFlex rightMotor = new SparkFlex(WristIDs.kDiffWristRightMotorID, MotorType.kBrushless);
              

    //Encoders

    public AbsoluteEncoder pitchEncoder;
    public AbsoluteEncoder rollEncoder;

    private double pitchPos;
    private double rollPos;

    private Pivot pivot;

    // Variables
    public static boolean runPID = true;
    public WristState lastState = WristState.TRANSITIONSTATE;
    /* ----- Initialization ----- */

    public Wrist() {
        pitchPID.enableContinuousInput(0, 1);
        rollPID.enableContinuousInput(0, 1);
        pivot = Pivot.getInstance();

        //Telemetry 
        SmartDashboard.putBoolean("Reefscape/DiffWrist/RunPID?", runPID);

        //Motor Configuration
        MotorR = new SparkMax(Ports.wristR, MotorType.kBrushless); 
        MotorL = new SparkMax(Ports.wristL, MotorType.kBrushless);

        MotorConfigL = new SparkMaxConfig();
        MotorConfigR = new SparkMaxConfig();

        feedforwardR = new ArmFeedforward(0, .3,0);
        feedforwardL = new ArmFeedforward(0, .3, 0);
        // el: 88.2 piv: 56.5 rt:
        MotorConfigL
                .inverted(true)
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(60);
        MotorConfigL.absoluteEncoder
                .positionConversionFactor(1)
                .inverted(false);
        MotorConfigL.closedLoop
                .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                .outputRange(-1, 1)
                .positionWrappingEnabled(true);
        MotorL.configure(MotorConfigL, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        MotorConfigR
                .inverted(false)
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(60);
        MotorConfigR.absoluteEncoder
                .positionConversionFactor(1)
                .inverted(true);
        MotorConfigR.closedLoop
                .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                .outputRange(-1, 1)
                .positionWrappingEnabled(true);
        MotorR.configure(MotorConfigR, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        //Diff Wrist Start point
        rollEncoder = MotorR.getAbsoluteEncoder(); //Max: 0.35, 0.8    positions to go to: Score: .75, hold: .5
        pitchEncoder = MotorL.getAbsoluteEncoder(); //Max: .75, .16   Positions to go to:  Grab: .7,  Score: .2   hold: .45
    
       
        
        if (runPID) {
            pitchPID.setSetpoint(0.22);//Look over this later. yes you gotta look over this
            rollPID.setSetpoint(0);
        }
        
                
    }

    /* ----- Updaters ----- */

    /**
     * Will update the volts to use calculated by the PID
     * @param pos current position
     */
    public void updatePID(double pitchPos, double rollPos) {
        double pitchVoltage = pitchPID.calculate(pitchPos);
        double rollVoltage = rollPID.calculate(rollPos);
        double angle = pivot.pivotMotor.getAbsoluteEncoder().getPosition() - 12.3;
        if(angle < 0){
            angle += 360.0;
        }
        //Pitch voltage is being multiplied by 3 due to the fact that its on a 3:1 gear ration (3 times slower than roll)
        pitchVoltage *= 3;

        //  lVolts = pitchVoltage - rollVoltage;
        //  rVolts = pitchVoltage + rollVoltage;

        //L has the pitch encoder
        //R has the roll encoder
        //  lVolts =feedforwardL.calculate(MotorL.getAbsoluteEncoder().getPosition(), 0);
        //  rVolts =feedforwardR.calculate(MotorL.getAbsoluteEncoder().getPosition(), 0);
        
        lVolts = -pitchVoltage - rollVoltage - feedforwardL.calculate(2*Math.PI* MotorL.getAbsoluteEncoder().getPosition(), 0)
        + pivot.feedForward.calculate(Math.toRadians(angle),0);
        rVolts = -pitchVoltage + rollVoltage - feedforwardR.calculate(2*Math.PI*MotorL.getAbsoluteEncoder().getPosition(), 0)
        + pivot.feedForward.calculate(Math.toRadians(angle),0);
        lVolts = MathUtil.clamp(lVolts, -14, 14);
        rVolts = MathUtil.clamp(rVolts, -14, 14);
        setVoltage(lVolts, rVolts);
    }

    // @Override
    public void periodic() {
        runPID = true;
        runPID = SmartDashboard.getBoolean("Reefscape/DiffWrist/RunPID?", true);
        pitchPos = pitchEncoder.getPosition();
        rollPos = rollEncoder.getPosition();
        if (runPID) {
            updatePID(pitchPos, rollPos);
        }

        // MotorL.setVoltage(feedforwardL.calculate(MotorL.getAbsoluteEncoder().getPosition(), 0));
        // MotorR.setVoltage(feedforwardR.calculate(MotorL.getAbsoluteEncoder().getPosition(), 0));

        
    }
    public double getVoltageLeft(){
        return lVolts;
    }
    public double getVoltageRight(){
        return rVolts;
    }
    public static Wrist getInstance() {
        if (DW == null) {
            DW = new Wrist();
        }
        return DW;
    }

    /**
     * Finds the current encoder value for the wrist's pitch
     * @return the current encoder value for pitch in degrees
     */
    public double getPitchAngle() {
        return pitchPos * 360;
    }

    /**
     * Finds the current encoder value for the wrist's roll
     * @return the current encoder value for roll in degrees
     */
    public double getRollAngle() {
        return rollPos * 360;
    }

    /**
     * Sets both of the motors is the Diff Wrist system to same voltage
     * Temporary way of usage, use till deemed safe to use a PID
     * @param leftVoltage voltage to set left motor to
     * @param rightVoltage voltage to set right motor to
     */
    public void setVoltage(double leftVoltage, double rightVoltage) {
        MotorL.setVoltage(leftVoltage);
        MotorR.setVoltage(rightVoltage);
    }

    /**
     * sets the target position of the pitch PID
     * @param setpoint
     */
    public void setPitchSetpoint(double setpoint) {
        setpoint /= 360;
        pitchPID.setSetpoint(setpoint);
    }

    public boolean atPitchSetpoint() {
        double pitchAngle = getPitchAngle();
        double pitchSetpoint = getPitchSetpoint(); // was 0.13
        if ((pitchAngle > pitchSetpoint - 0.13) && (pitchAngle < pitchSetpoint + 0.13)) {
            return true;
        }
        return false;
    }

    /**
     * sets the target position of the roll PID
     * @param setpoint
     */
    public void setRollSetpoint(double setpoint) {
        setpoint /= 360;
        rollPID.setSetpoint(setpoint);
    }

    public boolean atRollSetpoint() {
        double rollAngle = getRollAngle();
        double rollSetpoint = getRollSetpoint();
        if ((rollAngle > rollSetpoint - 0.01) && (rollAngle < rollSetpoint + 0.01)) {
            return true;
        }
        return false;
    }

    /**
     * Used for getting the current target of the Pitch
     * @return angle of pitch
     */
    public double getPitchSetpoint() {
        return pitchPID.getSetpoint() * 360;
    }

    /**
     * Used for getting the current target of the Roll
     * @return angle of roll
     */
    public double getRollSetpoint() {
        return rollPID.getSetpoint() * 360;
    }

    

}