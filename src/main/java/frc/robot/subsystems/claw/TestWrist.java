// package frc.robot.subsystems.claw;

// import com.revrobotics.spark.SparkMax;
// import com.revrobotics.spark.SparkBase.PersistMode;
// import com.revrobotics.spark.SparkBase.ResetMode;
// import com.revrobotics.AbsoluteEncoder;
// import com.revrobotics.spark.SparkLowLevel.MotorType;
// import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
// import com.revrobotics.spark.config.SparkMaxConfig;

// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.math.controller.ArmFeedforward;
// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import frc.robot.Ports;
// import frc.robot.subsystems.pivot.Pivot;

// import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

// public class TestWrist  {
//     public static WristState wristState = WristState.TRANSITIONSTATE;

//     public enum WristState {
//         GROUND(0,0), 
//         LOWALGAEINTAKE(0,0),
//         HIGHALGAEINTAKE(0,0),
//         L1CORALSCORE(0,0),
//         L2CORALSCORE(0,0),
//         L3CORALSCORE(0,0),
//         L4CORALSCORE(0,0), 
//         PROCESSOR(0,0),
//         HUMANSTATIONINTAKE(0,0),
//         TRANSITIONSTATE(0,  10),
//         TEST(0,0),
//         CLIMB(0,0),
//         NET(0,0);
//         public double rotate;
//         public double tilt;
//         private WristState(double rotate, double tilt) {
//             this.rotate = rotate;
//             this.tilt = tilt;
            
//         }
//     }
//     public double lVolts;
//     public double rVolts;
//     public SparkMax MotorR;
//     public SparkMax MotorL;
//     public SparkMaxConfig MotorConfigL;
//     public SparkMaxConfig MotorConfigR;
//     private static Wrist DW;
//     public ArmFeedforward feedforwardL;
//     public ArmFeedforward feedforwardR;
    
//     // PID
//     //OLD
//     // public PIDController pitchPID = new PIDController(13, 0, 2);
//     // public PIDController rollPID = new PIDController(70, 0, 0);
    
//     public PIDController pitchPID = new PIDController(0, 0, 0);
//     public PIDController rollPID = new PIDController(0., 0, 0);

//     // // Motors
//     // private SparkFlex leftMotor = new SparkFlex(WristIDs.kDiffWristLeftMotorID, MotorType.kBrushless);
//     // private SparkFlex rightMotor = new SparkFlex(WristIDs.kDiffWristRightMotorID, MotorType.kBrushless);
              

//     //Encoders

//     public AbsoluteEncoder pitchEncoder;
//     public AbsoluteEncoder rollEncoder;

//     public IdleMode mode = IdleMode.kBrake;

//     private double pitchPos;
//     private double rollPos;

//     private Pivot pivot;

//     // Variables
//     public static boolean runPID = true;
//     public WristState lastState = WristState.TRANSITIONSTATE;

//     public TestWrist() {
//         pitchPID.enableContinuousInput(0, 1);
//         rollPID.enableContinuousInput(0, 1);
//         pivot = Pivot.getInstance();

//         //Telemetry 
//         SmartDashboard.putBoolean("Reefscape/DiffWrist/RunPID?", runPID);

//         //Motor Configuration
//         MotorR = new SparkMax(Ports.wristR, MotorType.kBrushless); // SAY THE CORRECT PORT NUMBER LATER
//         MotorL = new SparkMax(Ports.wristL, MotorType.kBrushless); // SAY THE CORRECT PORT NUMBER LATER

//         MotorConfigL = new SparkMaxConfig();
//         MotorConfigR = new SparkMaxConfig();

//         // feedforwardR = new ArmFeedforward(0, .37,0);
//         // feedforwardL = new ArmFeedforward(0, .37, 0);

//         feedforwardR = new ArmFeedforward(0, 0.1,0);
//         feedforwardL = new ArmFeedforward(0, 0.1, 0);

//         // el: 88.2 piv: 56.5 rt:
//         MotorConfigL
//                 .inverted(false)
//                 .idleMode(mode)
//                 .smartCurrentLimit(60);
//         MotorConfigL.absoluteEncoder
//                 .positionConversionFactor(360)
//                 .inverted(false);
//         MotorConfigL.closedLoop
//                 .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
//                 .outputRange(-1, 1)
//                 .positionWrappingEnabled(false);
//         MotorL.configure(MotorConfigL, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

//         MotorConfigR
//                 .inverted(true)
//                 .idleMode(mode)
//                 .smartCurrentLimit(60);
//         MotorConfigR.absoluteEncoder
//                 .positionConversionFactor(360)
//                 .inverted(true);
//         MotorConfigR.closedLoop
//                 .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
//                 .outputRange(-1, 1)
//                 .positionWrappingEnabled(false);
//         MotorR.configure(MotorConfigR, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
//         //Diff Wrist Start point
//         rollEncoder = MotorR.getAbsoluteEncoder(); //Max: 0.35, 0.8    positions to go to: Score: .75, hold: .5
//         pitchEncoder = MotorL.getAbsoluteEncoder(); //Max: .75, .16   Positions to go to:  Grab: .7,  Score: .2   hold: .45
    
//         if (runPID) {
//             pitchPID.setSetpoint(wristState.tilt);//LOOK OVER THIS LATER
//             rollPID.setSetpoint(wristState.rotate);
//         }         
//     }

// }
