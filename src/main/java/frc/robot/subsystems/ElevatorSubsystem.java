// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase {
  /** Creates a new ElevatorSubsystem. */
  public static final TalonFX elevatorMotor = new TalonFX(Constants.ElevatorConstants.ElevatorMotorID);
  private static TalonFXConfiguration elevatorConfig = new TalonFXConfiguration();
  public ElevatorSubsystem() {
    resetElevator();
  }


  public void resetElevator(){
    elevatorMotor.setPosition(0);
    elevatorMotor.setNeutralMode(NeutralModeValue.Coast);
    elevatorConfig.Slot0.kP = Constants.ElevatorConstants.ElevatorkP;
    elevatorConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = Constants.ElevatorConstants.ElevatorClosedLoopRamp;
    elevatorMotor.getConfigurator().apply(elevatorConfig);
  }

  public static void setElevator(double position){
    PositionDutyCycle elveatorPosition = new PositionDutyCycle(0);
    elevatorMotor.setControl(elveatorPosition.withPosition(position));
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Elevator position", elevatorMotor.getPosition().getValueAsDouble());
  }
}
