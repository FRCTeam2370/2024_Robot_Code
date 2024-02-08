// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.TunerConstants;

/**
 * Basic simulation of a swerve subsystem with the methods needed by PathPlanner
 */
public class SwerveSubsystem extends SubsystemBase {
  private SwerveModule[] modules;
  private SwerveDriveKinematics kinematics;
  private SwerveDriveOdometry odometry;

  private static Pigeon2 gyro;
  
  private Field2d field = new Field2d();
  
  public SwerveSubsystem() {
    gyro = new Pigeon2(TunerConstants.kPigeonId);
    modules = new SwerveModule[]{
      new SwerveModule(TunerConstants.FrontLeft),
      new SwerveModule(TunerConstants.FrontRight),
      new SwerveModule(TunerConstants.BackLeft),
      new SwerveModule(TunerConstants.BackRight)
    };
    kinematics = TunerConstants.kDriveKinematics;
    odometry = new SwerveDriveOdometry(kinematics, gyro.getRotation2d(), getPositions());

    // Configure AutoBuilder
    AutoBuilder.configureHolonomic(
      this::getPose, 
      this::resetPose, 
      this::getSpeeds, 
      this::driveRobotRelative, 
      TunerConstants.pathFollowerConfig,
      () -> {
          // Boolean supplier that controls when the path will be mirrored for the red alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
              return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
      },
      this
    );

    // Set up custom logging to add the current path to a field 2d widget
    PathPlannerLogging.setLogActivePathCallback((poses) -> field.getObject("path").setPoses(poses));

    SmartDashboard.putData("Field", field);
  }

  @Override
  public void periodic() {
    // Update the simulated gyro, not needed in a real project
    //gyro.updateRotation(getSpeeds().omegaRadiansPerSecond);

    odometry.update(gyro.getRotation2d(), getPositions());

    field.setRobotPose(getPose());
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public void resetPose(Pose2d pose) {
    odometry.resetPosition(gyro.getRotation2d(), getPositions(), pose);
  }

  public ChassisSpeeds getSpeeds() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }

  public void driveFieldRelative(ChassisSpeeds fieldRelativeSpeeds) {
    driveRobotRelative(ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, getPose().getRotation()));
  }

  public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
    ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);

    SwerveModuleState[] targetStates = kinematics.toSwerveModuleStates(targetSpeeds);
    setStates(targetStates);
  }

  public void setStates(SwerveModuleState[] targetStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(targetStates, TunerConstants.maxSpeed);

    for (int i = 0; i < modules.length; i++) {
      modules[i].setDesiredState(targetStates[i]);
    }
  }

  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[modules.length];
    for (int i = 0; i < modules.length; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  public SwerveModulePosition[] getPositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[modules.length];
    for (int i = 0; i < modules.length; i++) {
      positions[i] = modules[i].getPosition();
    }
    return positions;
  }






  class SwerveModule{
    private TalonFX mdriveMotor;
    private TalonFX mturnMotor;
    private CANcoder angleEncoder;
    private Rotation2d lastAngle;

    public SwerveModule(SwerveModuleConstants swerveModuleConstants){
      this.mdriveMotor = new TalonFX(swerveModuleConstants.DriveMotorId);

      this.mturnMotor = new TalonFX(swerveModuleConstants.SteerMotorId);

      this.angleEncoder = new CANcoder(swerveModuleConstants.CANcoderId);

      lastAngle = getState().angle;
    }

    public Rotation2d getAngle(){
      return Rotation2d.fromDegrees(falconToDegrees(mturnMotor.getPosition().getValueAsDouble(), TunerConstants.kSteerGearRatio));
    }

    public SwerveModulePosition getPosition(){
      return new SwerveModulePosition(falconToMeters(mdriveMotor.getPosition().getValueAsDouble(),
      TunerConstants.wheelCircumference, TunerConstants.kDriveGearRatio),
      getAngle());
    }

    public SwerveModuleState getState(){
      return new SwerveModuleState(falconToMPS(mdriveMotor.getVelocity().getValueAsDouble(),
      TunerConstants.wheelCircumference, TunerConstants.kDriveGearRatio),
      getAngle());
    }

    public void setDesiredState(SwerveModuleState desiredState){
      SwerveModuleState.optimize(desiredState, getState().angle);
      setAngle(desiredState);
      double percentOutput = desiredState.speedMetersPerSecond / TunerConstants.maxSpeed;
      mdriveMotor.set(percentOutput);

    }

    private void setAngle(SwerveModuleState desiredState){
      PositionDutyCycle mPositionDutyCycle = new PositionDutyCycle(0);
      Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (TunerConstants.maxSpeed * 0.01) ? lastAngle : desiredState.angle);

      mturnMotor.setControl(mPositionDutyCycle.withPosition(angle.getRotations()));
      lastAngle = angle;
    }
  }
  /**
   * Basic simulation of a swerve module, will just hold its current state and not use any hardware
   */
  class SimSwerveModule {
    private SwerveModulePosition currentPosition = new SwerveModulePosition();
    private SwerveModuleState currentState = new SwerveModuleState();

    public SwerveModulePosition getPosition() {
      return currentPosition;
    }

    public SwerveModuleState getState() {
      return currentState;
    }

    public void setTargetState(SwerveModuleState targetState) {
      // Optimize the state
      currentState = SwerveModuleState.optimize(targetState, currentState.angle);

      currentPosition = new SwerveModulePosition(currentPosition.distanceMeters + (currentState.speedMetersPerSecond * 0.02), currentState.angle);
    }
  }

  /**
   * Basic simulation of a gyro, will just hold its current state and not use any hardware
   */
  class SimGyro {
    private Rotation2d currentRotation = new Rotation2d();

    public Rotation2d getRotation2d() {
      return currentRotation;
    }

    public void updateRotation(double angularVelRps){
      currentRotation = currentRotation.plus(new Rotation2d(angularVelRps * 0.02));
    }
  }

  public static double falconToDegrees(double positionCounts, double gearRatio){
    return positionCounts * (360 / (gearRatio * 2048));
  }

  public static double falconToMeters(double positionCounts, double circumference, double gearRatio){
    return positionCounts * (circumference / (gearRatio * 2048));
  }

  public static double falconToRPM(double velocityCounts, double gearRatio){
    double motorRPM = velocityCounts * (600 / 2048);
    double mechRPM = motorRPM / gearRatio;
    return mechRPM;
  }

  public static double falconToMPS(double velocityCounts, double circumference, double gearRatio){
    double wheelRPM = falconToRPM(velocityCounts, gearRatio);
    double wheelMPS = (wheelRPM * circumference) / 60;
    return wheelMPS;
  }
}