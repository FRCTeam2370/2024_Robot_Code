// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.stream.IntStream.IntMapMultiConsumer;

import javax.swing.text.Position;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileCommand;
import frc.robot.Constants;
import frc.robot.commands.Intake.intakeToggle;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  public static final TalonFX IntakeMotor = new TalonFX(Constants.IntakeConstants.IntakeMotorID);
  public static final AnalogInput IntakeNoteDetector = new AnalogInput(Constants.IntakeConstants.IntakeNoteDetectorChannel);
  public static final TalonFX PositionIntakeMotor = new TalonFX(Constants.IntakeConstants.PositionIntakeMotorID);
  private static PositionDutyCycle IntakeUpDown = new PositionDutyCycle(0);
  private static TalonFXConfiguration IntakePositionconfig = new TalonFXConfiguration();
  private static double lastIntakePosition;
  public static CANcoder IntakePoseEncoder = new CANcoder(Constants.IntakeConstants.IntakeAbsolutePoseID);
  private static CANcoderConfiguration IntakePoseConfig = new CANcoderConfiguration();
  private static FeedbackConfigs intakeFeedbackConfig = new FeedbackConfigs();
  private static PositionDutyCycle rollersPose = new PositionDutyCycle(IntakeMotor.getPosition().getValueAsDouble());
  private static TalonFXConfiguration rollersConfig = new TalonFXConfiguration();
  public static DigitalInput IntakeNoteDetector2 = new DigitalInput(4);
  public IntakeSubsystem() {
    resetIntake();
  }

  public static void LowerIntake(){
    //PositionIntakeMotor.setNeutralMode(NeutralModeValue.Brake);
    IntakePositionconfig.Slot0.kP = Constants.IntakeConstants.PositionIntakeMotorDownkP;
    IntakePositionconfig.Slot0.kI = 0; 
    IntakePositionconfig.Slot0.kV = 0.1;
    IntakePositionconfig.Slot0.kG = 0;
    IntakePositionconfig.Slot0.kD = 0;
    IntakePositionconfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = Constants.IntakeConstants.PositionIntakeMotorClosedLoopRamp;
    PositionIntakeMotor.getConfigurator().apply(IntakePositionconfig);
    PositionIntakeMotor.setControl(IntakeUpDown.withPosition(0.53));//Change this value when the motor is on the thing
  }

  public static void RiseIntake(){
    //PositionIntakeMotor.setNeutralMode(NeutralModeValue.Coast);
    IntakePositionconfig.Slot0.kP = Constants.IntakeConstants.PositionIntakeMotorUpkP;
    IntakePositionconfig.Slot0.kI = 0; 
    IntakePositionconfig.Slot0.kG = 0;
    IntakePositionconfig.Slot0.kD = 0;
    IntakePositionconfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = Constants.IntakeConstants.PositionIntakeMotorClosedLoopRamp;
    PositionIntakeMotor.getConfigurator().apply(IntakePositionconfig);
    PositionIntakeMotor.setControl(IntakeUpDown.withPosition(0.01));//Change this value as well
  }
  
  public static void setIntakePostition(double position){
    double kI = position < IntakePoseEncoder.getPosition().getValueAsDouble() ? 0 : 0;//0.18,   0.2, 0.5
    double kP = position < IntakePoseEncoder.getPosition().getValueAsDouble() ? 1.5 : 8.4;//3,   7, 7.75
    double kD = position < IntakePoseEncoder.getPosition().getValueAsDouble() ? 0 : 0.1;//0.02
    double kG = position < IntakePoseEncoder.getPosition().getValueAsDouble() ? 0.03 : 0.035;
    IntakePositionconfig.Slot0.kP = kP;
    IntakePositionconfig.Slot0.kI = kI;
    IntakePositionconfig.Slot0.kG = kG;//0.1,  0.25, 1
    IntakePositionconfig.Slot0.kD = kD;
    //IntakePositionconfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 0.01;
    PositionIntakeMotor.getConfigurator().apply(IntakePositionconfig);
    PositionIntakeMotor.setControl(IntakeUpDown.withPosition(position));
    //lastIntakePosition = PositionIntakeMotor.getPosition().getValueAsDouble();
  }
  

  public static void setAbsoluteIntakePose(double position){
    //double kI = position < PositionIntakeMotor.getPosition().getValueAsDouble() ? 0 : 0.18;
    //IntakePositionconfig.Slot0.kP = Constants.IntakeConstants.PositionIntakeMotorUpkP;
    //IntakePositionconfig.Slot0.kI = kI;
    IntakePositionconfig.Slot0.kV = 0.08;
    IntakePositionconfig.Slot0.kP = Constants.IntakeConstants.PositionIntakeMotorUpkP;
    IntakePositionconfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 0.01;
    PositionIntakeMotor.getConfigurator().apply(IntakePositionconfig);
    PositionIntakeMotor.setControl(IntakeUpDown.withPosition(position));
  }

  public static void resetIntake(){
    PositionIntakeMotor.setNeutralMode(NeutralModeValue.Brake);
    PositionIntakeMotor.setPosition(0);
    //IntakePositionconfig.Feedback.FeedbackRemoteSensorID = IntakePoseEncoder.getDeviceID();
    
    IntakePoseConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
    IntakePoseEncoder.getConfigurator().apply(IntakePoseConfig);

    intakeFeedbackConfig.FeedbackRemoteSensorID = IntakePoseEncoder.getDeviceID();
    intakeFeedbackConfig.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    IntakePositionconfig.withFeedback(intakeFeedbackConfig);

    PositionIntakeMotor.getConfigurator().apply(IntakePositionconfig);
    IntakeMotor.setNeutralMode(NeutralModeValue.Brake);
    rollersConfig.Slot0.kP = 1.9;//1.5
    IntakeMotor.getConfigurator().apply(rollersConfig);
    
    IntakePoseEncoder.setPosition(0);//IntakePoseEncoder.getAbsolutePosition().getValueAsDouble()-0.063721
    //IntakeMotor.getPosition().setUpdateFrequency(0);
    IntakeMotor.getVelocity().setUpdateFrequency(0);
    PositionIntakeMotor.getVelocity().setUpdateFrequency(0);
  }

  public static void ForwardIntake(){
    IntakeMotor.set(0.6);//0.5 if the bar is back where it was at the third closest to the back of the intake
  }

  public static void BackwardIntake(){
    IntakeMotor.set(-0.6);//-0.6
  }

  public static void storeIntake(){
    IntakePositionconfig.Slot0.kP = Constants.IntakeConstants.PositionIntakeMotorUpkP;
    PositionIntakeMotor.getConfigurator().apply(IntakePositionconfig);
    PositionIntakeMotor.setControl(IntakeUpDown.withPosition(0.18));
  }

  public static void runForABit(){
    IntakeMotor.setPosition(0);
    PositionDutyCycle pose = new PositionDutyCycle(IntakeMotor.getPosition().getValueAsDouble());
    IntakeMotor.setControl(pose.withPosition(IntakeMotor.getPosition().getValueAsDouble()+4));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Intake note detector", IntakeNoteDetector.getValue());
    SmartDashboard.putNumber("Intake Position Motor", PositionIntakeMotor.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Intake Relative Pose", IntakePoseEncoder.getPosition().getValueAsDouble());
    SmartDashboard.putBoolean("Intake Not Detector 1", IntakeNoteDetector2.get());
    SmartDashboard.putNumber("Rollers Intake Pose", IntakeMotor.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Intake Absolute Pose", IntakePoseEncoder.getAbsolutePosition().getValueAsDouble());
    //SmartDashboard.putNumber("Position Intake Error", );
    
  }
}