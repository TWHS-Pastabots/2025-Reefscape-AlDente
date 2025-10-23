package frc.robot.subsystems.elevator;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import frc.robot.Ports;

import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.IO.DigitalInputs;

import com.revrobotics.spark.config.SparkFlexConfig;


import edu.wpi.first.math.controller.ElevatorFeedforward;

import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

public class Elevator {
    public static double adjuster;
    public DigitalInputs bottomLimitSwitch;
    public DigitalInputs topLimitSwitch;
    public static Elevator instance;
    public SparkFlex elevatorMotorL;
    public SparkFlexConfig configL;
    public SparkFlex elevatorMotorR;
    public SparkFlexConfig configR;

    public SparkClosedLoopController ControllerR;
    public SparkClosedLoopController ControllerL;

    public  ElevatorFeedforward feedForward;
    public  ElevatorState elevatorState = ElevatorState.GROUND;
    public enum ElevatorState{
        GROUND(0),    
        LOWALGAEINTAKE(15),
        HIGHALGAEINTAKE(27),
        L1CORALSCORE(0),
        L2CORALSCORE(14),//13.5
        L3CORALSCORE(41.1),//39.5 
        L4CORALSCORE(76),
        PROCESSOR(9),
        HUMANSTATIONINTAKE(10.5),
        AUTONTRANSITION(9),
        TEST(60);

        public double position;
        private ElevatorState(double position){
            this.position = position;
        }
    }
    public Elevator() {
        elevatorMotorL = new SparkFlex(Ports.elevatorMotorL, MotorType.kBrushless); 
        elevatorMotorR = new SparkFlex(Ports.elevatorMotorR, MotorType.kBrushless); 
        configL = new SparkFlexConfig();
        configR = new SparkFlexConfig();

        configL
            .closedLoopRampRate(.5)
            .inverted(true)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(50);
        configL.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(ElevatorConstants.elevatorPCoefficient, ElevatorConstants.elevatorICoefficient, ElevatorConstants.elevatorDCoefficient)
            .outputRange(-1, 1);
         elevatorMotorL.configure(configL, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

         configR
            .closedLoopRampRate(.5
            )
            .inverted(false)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(50);
        configR.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(ElevatorConstants.elevatorPCoefficient, ElevatorConstants.elevatorICoefficient, ElevatorConstants.elevatorDCoefficient)
            .outputRange(-1, 1);
         elevatorMotorR.configure(configR, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        // kg: 0.2
        feedForward = new ElevatorFeedforward(0.16,0.23,0,0); //FILL THESE NUMBERS IN LATER

         ControllerL = elevatorMotorL.getClosedLoopController();
         ControllerR = elevatorMotorR.getClosedLoopController();

        topLimitSwitch = DigitalInputs.getInstance();
        bottomLimitSwitch = DigitalInputs.getInstance();

    }
    public void updatePose(){
        ControllerL.setReference(elevatorState.position, ControlType.kPosition, ClosedLoopSlot.kSlot0,
        feedForward.calculate(elevatorMotorL.getEncoder().getPosition(), 0));//SLOTS ONCE WE KNOW 

        ControllerR.setReference(elevatorState.position, ControlType.kPosition, ClosedLoopSlot.kSlot0,//SAME THING
        feedForward.calculate(elevatorMotorR.getEncoder().getPosition(), 0));

        if(getBottomLimitSwitch()){
            elevatorMotorL.getEncoder().setPosition(0);
            elevatorMotorR.getEncoder().setPosition(0);
        }
        
    }
    public void UpdateEnumPoses(){
        if(elevatorState == ElevatorState.GROUND){
            elevatorState.position = elevatorState.position;
        }else{
            elevatorState.position += adjuster;
        }
        
    }
    public void setElevatorPosition(double position){
        elevatorState.position = position;
    }
    public double getElevatorPosition(){
        return elevatorMotorL.getEncoder().getPosition();
    }
    public void setElevatorState(ElevatorState state){
        elevatorState = state;
    }
    public boolean hasReachedPose(double tolerance) {
        return Math.abs(getElevatorPosition() - elevatorState.position) < tolerance;
    }
    public ElevatorState getElevatorState(){
        return elevatorState;
    }
     public ElevatorState getState(){
        return this.elevatorState;
    }
    public void ElevatorGoUp(){
        elevatorMotorL.set(-.1);
        elevatorMotorR.set(-.1);
    }
    public void ElevatorStop(){
        elevatorMotorL.set(0);
        elevatorMotorR.set(0);
    }
    public void ElevatorGoDown(){
        elevatorMotorL.set(.1);
        elevatorMotorR.set(.1);
    }
    public static Elevator getInstance() {
        if (instance == null)
            instance = new Elevator();
        return instance;
    }
     public boolean getTopLimitSwitch(){
        return !topLimitSwitch.getInputs()[Ports.elevatorTop]; //MAKE SURE TO CHANGE THIS LATER
    }
    public boolean getBottomLimitSwitch(){
        return !bottomLimitSwitch.getInputs()[Ports.elevatorBottom]; //MAKE SURE TO CHANGE THIS LATER
    }
    
}

