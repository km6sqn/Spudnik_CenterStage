package org.firstinspires.ftc.teamcode.spudnik;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;


import java.util.ArrayList;
@Autonomous

public class RedParkLeft extends LinearOpMode{
   private DcMotorEx frontleft;
   private DcMotorEx backleft;
   private DcMotorEx frontright;
   private DcMotorEx backright;
  
    //imu stuff
    private BNO055IMU imu = null; //imu declaration
    private Orientation angles; //heading degree variable
    private double curHeading; //numerical heading in double form
    private Acceleration gravity; //acceleration
    private double accX; //numerical acceleration x
    private double accY; //numerical acceleration y
    private Acceleration overall; //overall acceleration
    private double overX; //numerical acceleration overall x
    private double overY; //numerical acceleration overall y
    private Position map; //position of robot on map
    private double mapX; //numerical map position x
    private double mapY; //numerical map position y
    
    
    private double leftPower= 0; //declare motor power on the left
    private double rightPower = 0; //declare motor power on the right
    @Override
    public void runOpMode(){
    initEverything();
    waitForStart();
    //turnRight(-90);
    strafeSomewhere(-100, 5000);
    goSomewhere(1980, 1000);
    }
    
    
    
    public void initEverything(){
        frontleft = hardwareMap.get(DcMotorEx.class, "frontleft");
        frontleft.setTargetPosition(0);
        frontleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontright = hardwareMap.get(DcMotorEx.class, "frontright");
        frontright.setTargetPosition(0);
        frontright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backleft = hardwareMap.get(DcMotorEx.class, "backleft");
        backleft.setTargetPosition(0);
        backleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backright = hardwareMap.get(DcMotorEx.class, "backright");
        backright.setTargetPosition(0);
        backright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        imu = hardwareMap.get(BNO055IMU.class, "imu"); //imu hardware class
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters(); //make new parameters
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES; //degree is the unit
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC; //in meters per second
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true; //logInput
        parameters.loggingTag          = "IMU"; //logs as IMU (see logcat)
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator(); //log acceleration
        imu.initialize(parameters); //initialize all parameters
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000); //calibrate the paramete
    }
        public void turnRight(int degrees){
        resetEncoders();

        getTelemetry();
        getNumbers();
        frontleft.setTargetPosition(-1000);
        frontright.setTargetPosition(-1000);
        backleft.setTargetPosition(-1000);
        backright.setTargetPosition(1000);
        frontleft.setVelocity(-2000);
        frontright.setVelocity(-2000);
        backleft.setVelocity(-2000);
        backright.setVelocity(2000);


        while(frontleft.isBusy() || frontright.isBusy()){
            getTelemetry();
            getNumbers();
            if(curHeading <= degrees - 50) {
                frontleft.setVelocity(-1000);
                frontright.setVelocity(-1000);
                backleft.setVelocity(-1000);
                backright.setVelocity(1000);
            }
            if(curHeading <= degrees - 40) {
                frontleft.setVelocity(-500);
                frontright.setVelocity(-500);
                backleft.setVelocity(-500);
                backright.setVelocity(500);
            }
            if(curHeading <= degrees - 5){
                frontleft.setVelocity(-10);
                frontright.setVelocity(-10);
                backleft.setVelocity(-10);
                backright.setVelocity(10);
            }
            if(curHeading <= degrees){
                break;
            }

        }

    }
    
     public void goSomewhere(int input, int speed){
        resetEncoders();
        frontleft.setTargetPosition(-input);
        frontright.setTargetPosition(input);
        backleft.setTargetPosition(-input);
        backright.setTargetPosition(-input);
        if(input < 0) {
            frontleft.setVelocity(speed);
            frontright.setVelocity(-speed);
            backleft.setVelocity(speed);
            backright.setVelocity(speed);
        }
        else{
            frontleft.setVelocity(-speed);
            frontright.setVelocity(speed);
            backleft.setVelocity(-speed);
            backright.setVelocity(-speed);
        }
        while(frontleft.isBusy() && frontright.isBusy()){
            getTelemetry();
            getNumbers();
        }
        resetEncoders();
     }
     
     public void strafeSomewhere(int input, int speed){
        resetEncoders();
        frontleft.setTargetPosition(-input);
        frontright.setTargetPosition(-input);
        backleft.setTargetPosition(input);
        backright.setTargetPosition(-input);
        if(input < 0) {
            frontleft.setVelocity(speed);
            frontright.setVelocity(speed);
            backleft.setVelocity(-speed);
            backright.setVelocity(speed);
        }
        else{
            frontleft.setVelocity(-speed);
            frontright.setVelocity(-speed);
            backleft.setVelocity(speed);
            backright.setVelocity(-speed);
        }
        while(frontleft.isBusy() && frontright.isBusy()){
            getTelemetry();
            getNumbers();
        }
        resetEncoders();
     }
     
     
    public void checkOrientation() {
        // read the orientation of the robot
        angles = this.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        this.imu.getPosition();
        // and save the heading
        curHeading = angles.firstAngle;
    }
    public void checkAcceleration(){
        gravity = this.imu.getAcceleration();
        accX = gravity.xAccel;
        accY = gravity.yAccel;
    }
    public void checkOverallAcceleration(){
        overall = this.imu.getOverallAcceleration();
        overX = overall.xAccel;
        overY = overall.yAccel;
    }
    public void checkNavigation(){
        map = this.imu.getPosition();
        mapX = map.x;
        mapY = map.y;
        telemetry.addData("X - Y Map", "X (%2f), Y (%2f)", mapX, mapY);
    }
    public int changeEncoder(int input){
        return (int)( ((input * 537.7) + .5) / 10);
    }
       public void resetEncoders(){
        frontleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontleft.setTargetPosition(0);
        frontright.setTargetPosition(0);
        backleft.setTargetPosition(0);
        backright.setTargetPosition(0);
        frontleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backright.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }
      public void getNumbers(){
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000); //reset calibration
        checkOrientation(); //check the orientation
        checkAcceleration(); //check acceleration/home/william/AndroidStudioProjects/6dayschedule/home/william/AndroidStudioProjects/6dayschedule
        checkOverallAcceleration(); //check overall acceleration
        checkNavigation(); //check navigation

    }
    public void getTelemetry(){
        telemetry.addData("Degrees", "* (%.2f)", curHeading); //degrees telemetry
        telemetry.addData("X - Y Acceleration", "X (%.2f), Y (%2f)", accX, accY); //acceleration telemetry
        telemetry.addData("X - Y Overall", "X (%.2f), Y (%2f)", overX, overY); //overall acceleration telemetry
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower); // wheel telemetry
        telemetry.addData("position", frontleft.getCurrentPosition());
        telemetry.addData("position", frontright.getCurrentPosition());
        telemetry.addData("isBusy", frontleft.isBusy());
        telemetry.update();


    }
}