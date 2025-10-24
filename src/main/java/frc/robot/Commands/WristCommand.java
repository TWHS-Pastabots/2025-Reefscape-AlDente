// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;


import frc.robot.subsystems.claw.Wrist;
import frc.robot.subsystems.claw.Wrist.WristState;

public class WristCommand extends Command {

    private Wrist DW = Wrist.getInstance();

    private double leftVoltage;
    private double rightVoltage;
    private double setpoint;
    private boolean pitchPID = false;
    private boolean ended;
    private double pitchPoint;
    private double rollPoint;
    private boolean runPID;
    private WristState lastState;
    /**
     * Sets the voltage of both motors on the Differental Wrist
     * @param leftVoltage voltage for the left motor to be set to
     * @param rightVoltage Voltage for the right motor to be set to
     */
    public WristCommand(double leftVoltage, double rightVoltage) {
        pitchPID = false;
        this.leftVoltage = leftVoltage;
        this.rightVoltage = rightVoltage;
    }

    /**
     * Will set one of the two PIDs on the Different wrist to the given setpoint
     * @param setpoint setpoint to go to
     * @param pitchPID True if for the roll PID, false if for the Yaw pid
     */
    public WristCommand(double setpoint, boolean pitchPID) {
        this.setpoint = setpoint;
        this.pitchPID = pitchPID;
    }
    public WristCommand(WristState state){
        lastState = state;
        this.pitchPoint = state.tilt;
        this.rollPoint = state.rotate;
        this.runPID = true;
    }
    public WristCommand(double pitchPoint, double rollPoint, boolean runPID){
        this.pitchPoint = pitchPoint;
        this.rollPoint = rollPoint;
        this.runPID = runPID;
    }
    @Override
    public void initialize() {
      ended = false;
        if (Wrist.runPID && runPID) {
            // if (pitchPID) {
            //     DW.setPitchSetpoint(setpoint);
            // } else {
            //     DW.setRollSetpoint(setpoint);
            // }
            DW.setPitchSetpoint(pitchPoint);
            DW.setRollSetpoint(rollPoint);
        } else {
            DW.setVoltage(leftVoltage, rightVoltage);
        }
        //DW.setPitchSetpoint(30);
    }
    public void execute(){
      DW.periodic();
      // if(pitchPID && DW.atPitchSetpoint()){
      //   ended = true;
      // }
      // else if(DW.atRollSetpoint()){
        ended = true;
      // }
      if(DW.atPitchSetpoint() && DW.atRollSetpoint())
      {
        ended = true;
      }
     
    }
    public void end(boolean interrupted) {
      
    }
    public void setLastState(){
        DW.lastState = lastState;
    }
    @Override
    public boolean isFinished() {
        return ended;
    }

}