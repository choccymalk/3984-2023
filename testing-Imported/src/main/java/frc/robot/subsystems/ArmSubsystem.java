package frc.robot.subsystems;
import org.opencv.core.Mat;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.Constants.Swerve.arm;


public class ArmSubsystem extends SubsystemBase{
    private CANSparkMax shoulderMotor; 
    private CANSparkMax jointMotor;
    public RelativeEncoder EncoderShoulder;
    public RelativeEncoder EncoderJoint;
    public ArmFeedforward ShoulderFF;
    public ArmFeedforward JointFF;
    public SparkMaxPIDController ShoulderPID;
    public SparkMaxPIDController JointPID;

    // Initialize the goal point for the arm.
    //private double[] goal = new double[2];

    
    public ArmSubsystem() {
        // Initialize feedforwards with the Ks Kg and Kv values.
        ShoulderFF = new ArmFeedforward(
            arm.Shoulder.Ks, 
            arm.Shoulder.Kg, 
            arm.Shoulder.Kv
        );
        JointFF = new ArmFeedforward(
            arm.Joint.Ks,
            arm.Joint.Kg, 
            arm.Joint.Kv
        );
        // Initialize the Motors
        shoulderMotor = new CANSparkMax(
            arm.Shoulder.rotMotorID, 
            MotorType.kBrushless
        );
        jointMotor = new CANSparkMax(
            arm.Joint.rotMotorID, 
            MotorType.kBrushless
        );
        // Initialize the built in motor encoders 
        EncoderShoulder = shoulderMotor.getEncoder(
            SparkMaxRelativeEncoder.Type.kHallSensor, 
            42
        );
        // Set the position to zero
        EncoderShoulder.setPosition(0);
        EncoderShoulder.setPositionConversionFactor(
            360/arm.Shoulder.gearRatio // in degrees
        );
        EncoderShoulder.setVelocityConversionFactor(
            (360 / arm.Shoulder.gearRatio) / 60.0 // in degrees/second
        );
        EncoderJoint = jointMotor.getEncoder(
            SparkMaxRelativeEncoder.Type.kHallSensor, 
            42
        );
        EncoderJoint.setPosition(0);
        EncoderJoint.setPositionConversionFactor(
            360/arm.Joint.gearRatio
        );
        EncoderShoulder.setVelocityConversionFactor(
            (360 / arm.Shoulder.gearRatio) / 60.0
        );
        // Initialize the built in Motor PID controllers
        ShoulderPID = shoulderMotor.getPIDController();
        ShoulderPID.setP(arm.Shoulder.Kp);
        ShoulderPID.setI(arm.Shoulder.Ki);
        ShoulderPID.setD(arm.Shoulder.Kd);

        JointPID = jointMotor.getPIDController();
        JointPID.setP(arm.Joint.Kp);
        JointPID.setI(arm.Joint.Ki);
        JointPID.setD(arm.Joint.Kd);
        
    }
    // Gets Encoder Positions
    public double[] getPositions(){
         // index 0 is shoulderPos, index 1 is jointPos
        double[] poss = new double[]{EncoderShoulder.getPosition(), EncoderJoint.getPosition()};
        return poss;
    }
    // Gets the errors  from position to target
    public double[] getErrors(double targetJoint, double targetShoulder){
        double[] errors = new double[2];
        double errorShoulder = targetShoulder - getPositions()[0];
        double errorJoint = targetJoint - getPositions()[1];
        errors[0] = errorShoulder;
        errors[1] = errorJoint;
        return errors;
    }

    public void stopArm(){
        shoulderMotor.set(0);
        jointMotor.set(0);
    }

    public void moveToAngle(double angleShoulder, double angleJoint){
        ShoulderPID.setReference(
            angleShoulder,
            ControlType.kPosition,
            0,
            ShoulderFF.calculate(
                (angleShoulder/360) * 2 * Math.PI, 
                0
            )
        );
        JointPID.setReference(
            angleJoint,
            ControlType.kPosition, 
            0, 
            JointFF.calculate(
                (angleJoint/360) * 2 * Math.PI, 
                0
            )
        );

    }
    public boolean atPosShoulder(double angle){
        return (Math.abs(getErrors(0, angle)[0]) < 1);
    }
    public boolean atPosJoint(double angle){
        return (Math.abs(getErrors(angle, 0)[1]) < 1);
    }

    // Gets the Angles required to get to a point. 
    public Rotation2d[] getAngles(double x, double y){
        double AngleShoulder = 0;
        double AngleJoint = 0;
        Rotation2d[] angles = new Rotation2d[2];
        double arm1Length = 34;
		double jointLength = 23.5;
        
		double hypotenuse = Math.hypot(x, y);
        // scales point to reachable range
        if (hypotenuse > arm1Length + jointLength){
            x = x/hypotenuse;
            x = x * (arm1Length + jointLength);
            y = y/hypotenuse;
            y = y * (arm1Length + jointLength);
        }
		AngleJoint = Math.pow(hypotenuse,  2) - Math.pow(arm1Length, 2) - Math.pow(jointLength, 2);
		AngleJoint = AngleJoint/(-2*arm1Length*jointLength);
		AngleJoint = Math.acos(AngleJoint);
        double theta2 = Math.pow(jointLength,2)-Math.pow(arm1Length,2)-Math.pow(hypotenuse,2);
        theta2 = theta2/(-2*arm1Length*hypotenuse);
        double j = Math.atan(y/x);
        double k = theta2-j;

        AngleShoulder = ((Math.PI/2)-k);
        //convert angle to radians
        angles[0] = new Rotation2d(AngleShoulder);
        angles[1] = new Rotation2d(AngleJoint);
        return angles;
    }

    // Creates a command that moves to a point x, y. If input = 0, 0 -> retracted mode.
    public Command moveTo(double x, double y){
        double degreesJoint;
        double degreesShoulder;
        if (x == 0 && y == 0){
            // 0, 0 means retracted mode means angles = 0;
            degreesShoulder = 0;
            degreesJoint = 0;
        }
        else{
            degreesShoulder = getAngles(x, y)[0].getDegrees();
            degreesJoint = getAngles(x, y)[1].getDegrees();
        }
        
        SmartDashboard.putNumber("Shoulder Goal", getAngles(x, y)[0].getDegrees());
        SmartDashboard.putNumber("Joint Goal", getAngles(x, y)[1].getDegrees());
        return run(() -> {
            moveToAngle(degreesShoulder, degreesJoint);
        }).until(() -> (atPosShoulder(degreesShoulder) /*&& atPosJoint(degrees)*/));
    }

    public void periodic(){
        SmartDashboard.putNumber("Shoulder", getPositions()[0]);
        SmartDashboard.putNumber("Joint", getPositions()[1]);
    }
    /*public void getNewPoint(){
        goal[0] = goal[0] + 0.05;
        goal[1] = goal[0] + 0.05;
    }*/
}
