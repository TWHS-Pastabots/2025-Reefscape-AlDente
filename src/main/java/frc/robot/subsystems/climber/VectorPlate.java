package frc.robot.subsystems.climber;


import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.MAXMotionConfig;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.Ports;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;

public class VectorPlate {
    // public PIDController pid = new PIDController(VectorConstants.vectorPCoefficient, 
    //                                             VectorConstants.vectorICoefficient, 
    //                                             VectorConstants.vectorDCoefficient);
    public SparkMax vectorMotor;
    private SparkMaxConfig config;
    public ArmFeedforward feedForward;
    private SparkClosedLoopController vectorController;
    private static VectorState vectorState = VectorState.DOWN;

    private static VectorPlate instance;
//erm change this joint later tee hee
    public enum VectorState {
        TEST(0),
        //0
        UP(1),
        DOWN(2);

        public double position;
        
        
        private VectorState(double position) {
            this.position = position;
            
        }
    }

    public VectorPlate() {
        //pid.enableContinuousInput(0, 180);
        vectorMotor = new SparkMax(Ports.vector, MotorType.kBrushless);
        config = new SparkMaxConfig();

        config
            .closedLoopRampRate(.5)
            .inverted(false)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(50);
        config.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(0,0,0)
            .outputRange(-1, 1);
         vectorMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        // config.closedLoop.maxMotion
        //     .allowedClosedLoopError(2)
        //     .maxVelocity(12000)
        //     .maxAcceleration(108000);
      
        vectorMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        vectorController = vectorMotor.getClosedLoopController();
        feedForward = new ArmFeedforward(0, 0, 0, 0);
        //was .73\
    }
    public void updatePose() {
        // vectorMotor.setVoltage(pid.calculate(vectorMotor.getAbsoluteEncoder().getPosition(), getState().position) 
        // );

        double angle = vectorMotor.getEncoder().getPosition();
        // if(angle < 0){
        //     angle += 360.0;
        // }

        //+ feedForward.calculate(vectorMotor.getAbsoluteEncoder().getPosition(),0)
        vectorController.setReference(vectorState.position, ControlType.kPosition, ClosedLoopSlot.kSlot0,
                feedForward.calculate(Math.toRadians(angle), 0));
       
        //vectorController.setReference(vectorState.position, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0);
    }
    public void setVectorPosition(double position)
    {   
        vectorState.position = position;
    }
    public double getPosition() {
        return vectorMotor.getEncoder().getPosition();
    }
    public void setVectorState(VectorState state){
        vectorState = state;
    }
    public boolean hasReachedPose(double tolerance) {
        return Math.abs(getPosition() - vectorState.position) < tolerance;
    }
    public VectorState getState(){
        return this.vectorState;
    }
    public static VectorPlate getInstance() {
        if (instance == null)
            instance = new VectorPlate();
        return instance;
    }
}
