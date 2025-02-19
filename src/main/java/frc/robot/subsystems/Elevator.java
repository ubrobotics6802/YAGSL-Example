// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.*;
import com.revrobotics.servohub.ServoChannel;
import com.revrobotics.servohub.ServoChannel.ChannelId;
import com.revrobotics.servohub.ServoHub;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase {
  /** Creates a new Elevator. */
  private SparkMax elevatorMotorLeft;
  private SparkMax elevatorMotorRight;
  // Initialize the servo hub
  ServoHub m_servoHub = new ServoHub(2);

  // Obtain a servo channel controller
  ServoChannel m_channel0 = m_servoHub.getServoChannel(ChannelId.kChannelId5);
  public Elevator(SparkMaxConfig config) {    
    m_channel0.setPowered(true);
    m_channel0.setEnabled(true);
    
config.closedLoop
    .p(0.04)
    .i(0)
    .d(0)
    .outputRange(-1, 1);



    SparkMaxConfig elevatorFollowerConfig = new SparkMaxConfig();
    elevatorFollowerConfig
        .smartCurrentLimit(40)
        .idleMode(IdleMode.kBrake).inverted(true);
    elevatorFollowerConfig.follow(ElevatorConstants.ELEVATOR_MOTOR_RIGHT_ID);
    elevatorMotorLeft = new SparkMax(ElevatorConstants.ELEVATOR_MOTOR_LEFT_ID, MotorType.kBrushless);
    elevatorMotorRight = new SparkMax(ElevatorConstants.ELEVATOR_MOTOR_RIGHT_ID, MotorType.kBrushless);
    elevatorMotorLeft.configure(elevatorFollowerConfig, ResetMode.kResetSafeParameters, null);
    elevatorMotorRight.configure(config, ResetMode.kResetSafeParameters, null);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setPosition(int position) {
    m_channel0.setPulseWidth(position);

  }

public void setPower(double power) {
  elevatorMotorRight.set(power);
}  

  public void setElevatorPosition(double position) {
    elevatorMotorRight.getClosedLoopController().setReference(position, ControlType.kPosition);
    System.out.println(position);
  }
}
