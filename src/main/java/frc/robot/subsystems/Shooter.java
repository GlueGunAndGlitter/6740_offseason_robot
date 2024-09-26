// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.compound.Diff_DutyCycleOut_Position;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class Shooter extends SubsystemBase {
  CANSparkFlex frontUpMotor = new CANSparkFlex(Constants.ShooterConstants.FRONT_UP_MOTOR_ID,
  MotorType.kBrushless);
  CANSparkFlex frontDownMotor = new CANSparkFlex(Constants.ShooterConstants.FONT_DOWN_MOTOR_ID,
  MotorType.kBrushless);
  CANSparkFlex backDownMotor = new CANSparkFlex(Constants.ShooterConstants.BACK_DOWN_MOTOR_ID,
  MotorType.kBrushless);


  
  /** Creates a new Shooter. */
  
  public Shooter() {
    
    frontDownMotor.setInverted(true);


  }
   
  private void shootUp(){
    frontUpMotor.set(Robot.ShooterfrontUpMotorSpeed.getDouble(0));
    frontDownMotor.set(Robot.ShooterfrontDownMotorSpeed.getDouble(0));
    backDownMotor.set(Robot.ShooterBackDownMotorSpeed.getDouble(0));
  }

  private void input(){
    frontUpMotor.set(-Robot.ShooterfrontUpMotorSpeed.getDouble(0)-0.5);
    frontDownMotor.set(-Robot.ShooterfrontDownMotorSpeed.getDouble(0)-0.5);
    backDownMotor.set(-Robot.ShooterBackDownMotorSpeed.getDouble(0)-0.5);
  }

  private void floorInput(){
    frontUpMotor.set(-Robot.ShooterfrontUpMotorSpeed.getDouble(0) + 0.3);
    frontDownMotor.set(Robot.ShooterfrontDownMotorSpeed.getDouble(0) - 0.3);
    backDownMotor.set(-Robot.ShooterBackDownMotorSpeed.getDouble(0) + 0.3);
  }

  
  public Command shootUpCommand(){
    return this.run(() -> shootUp());
  }

  public Command inputCommand(){
    return this.run(() -> input());
  }

  public Command floorInputCommand(){
    return this.run(() -> floorInput());
  }
  
  
  //public Command intakeShooterCommand(){
   // return this.run(() -> zeroMotors(1));
 // }
  
  private void stopMotors(){
    frontUpMotor.stopMotor();
    frontDownMotor.stopMotor();
    backDownMotor.stopMotor();
    
  }
  
  
  public Command stopMotorsCommand(){
    return this.run(() -> stopMotors());
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
  }
}
