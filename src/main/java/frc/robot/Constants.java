// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** Add your docs here. */
public class Constants {
    
    public class IntakeConstants{
        public static final int IntakeMotorID = 6;
        public static final int IntakeNoteDetectorChannel = 1;
        public static final int PositionIntakeMotorID = 13;
        public static final double PositionIntakeMotorUpkP = 0.05;
        public static final double PositionIntakeMotorDownkP = 0.03;
        public static final double PositionIntakeMotorClosedLoopRamp = 0.5;
    }

    public class ShooterConstants{
        public static final int ShooterMotor1ID = 15;
        public static final int ShooterMotor2ID = 16;
        public static final int ShooterAimCANcoderID = 12;
        public static final int ShooterAimMotorID = 1;
        public static final int ShooterNoteDetectorChannel = 0;
        public static final double ShooterClosedLoopRamp = 1;
        public static final double ShooterAimMotorkP = 0.01;
    }

    public class ElevatorConstants{
        public static final int ElevatorMotorID = 14;
        public static final double ElevatorkP = 0.06;
        public static final double ElevatorClosedLoopRamp = 0.1;
    }

    public class ClimberConstants{
        public static final int ClimberMotorID = 8;
        public static final double ClimberkP = 0.1;
        public static final double ClimberClosedLoopRamp = 0.5;
    }
    public class LimelightConstants{
        //In inches
        public static double limelightMoundHeight = 17;
        //In degrees
        public static double limelightMountPitch = 15;

    }

}
