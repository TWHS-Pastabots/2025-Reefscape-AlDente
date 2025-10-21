package frc.robot.subsystems.pivot;


import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.Ports;
import frc.robot.Constants.PivotConstants;

public class Pivot {
    // public PIDController pid = new PIDController(PivotConstants.pivotPCoefficient, 
    //                                             PivotConstants.pivotICoefficient, 
    //                                             PivotConstants.pivotDCoefficient);
    public SparkMax pivotMotor;
    private SparkMaxConfig config;
    public ArmFeedforward feedForward;
    private SparkClosedLoopController pivotController;
    private static PivotState pivotState = PivotState.TRANSITIONSTATE;

    private static Pivot instance;
//erm change this joint later tee hee
    public enum PivotState {
        GROUND(0),
        //0
        LOWALGAEINTAKE(145.3), //was 40.3
        HIGHALGAEINTAKE(128.5), // was 50.5
        L1CORALSCORE(106.556),
        L2CORALSCORE(110),
        L3CORALSCORE(107),
        L4CORALSCORE(101),//103
        PROCESSOR(38),
        HUMANSTATIONINTAKE(78), //was 80.289
        TRANSITIONSTATE(100),
        SHOOTINGNET(80),//changed it to 105
        HALFGROUND(55),//test
        SIGMATEST(120.3),
        CLIMB(35);

        public double position;
        
        
        private PivotState(double position) {
            this.position = position;
            
        }
    }

    public Pivot() {
        //pid.enableContinuousInput(0, 180);
        pivotMotor = new SparkMax(Ports.pivot, MotorType.kBrushless);
        config = new SparkMaxConfig();
        config
        .closedLoopRampRate(1.1) //was 1 (1.3 state)
            .inverted(true)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(60);
        config.absoluteEncoder
            .inverted(true)
            .positionConversionFactor(360);
        config.closedLoop
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
            .pid(0.008, 0., 0.2)
            .positionWrappingEnabled(true)
            .positionWrappingInputRange(0, 360)
            .outputRange(-1, 1);
        // config.closedLoop.maxMotion
        //     .allowedClosedLoopError(2)
        //     .maxVelocity(12000)
        //     .maxAcceleration(108000);
      
        pivotMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        pivotController = pivotMotor.getClosedLoopController();
        feedForward = new ArmFeedforward(0, 0.62, 0, 0);

    }
    public void updatePose() {
        // pivotMotor.setVoltage(pid.calculate(pivotMotor.getAbsoluteEncoder().getPosition(), getState().position) 
        // );
        double angle = pivotMotor.getAbsoluteEncoder().getPosition() - 13.5;
        if(angle < 0){
            angle += 360.0;
        }
        //+ feedForward.calculate(pivotMotor.getAbsoluteEncoder().getPosition(),0)
        pivotController.setReference(pivotState.position, ControlType.kPosition, ClosedLoopSlot.kSlot0,
                feedForward.calculate(Math.toRadians(angle), 0));
       
        //pivotController.setReference(pivotState.position, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0);
    }
    public void setPivotPosition(double position)
    {   
        pivotState.position = position;
    }
    public double getPosition() {
        return pivotMotor.getAbsoluteEncoder().getPosition();
    }
    public void setPivotState(PivotState state){
        pivotState = state;
    }
    public boolean hasReachedPose(double tolerance) {
        return Math.abs(getPosition() - pivotState.position) < tolerance;
    }
    public PivotState getState(){
        return this.pivotState;
    }
    public static Pivot getInstance() {
        if (instance == null)
            instance = new Pivot();
        return instance;
    }
    
}
