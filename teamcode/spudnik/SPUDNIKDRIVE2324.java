package org.firstinspires.ftc.teamcode.spudnik;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.JavaUtil;

@TeleOp(name = "SPUDNIKDRIVE2324")
public class SPUDNIKDRIVE2324 extends LinearOpMode {

  private DcMotor intake;
  private DcMotor pull;

  private Servo launcher;
  private Servo claw;
  private Servo wrist;
  private Servo pullWrist;

  int drivePowerFactor;
  int goToPosition;
  
  private DcMotor armleft;
  private DcMotor armright;
  
  private DcMotor frontleft;
  private DcMotor backleft;
  private DcMotor frontright;
  private DcMotor backright;
  
  
  @Override
  public void runOpMode() {

    // Put initialization blocks here.
    initEverything();
    waitForStart();
      while (opModeIsActive()) {
        drive();
        pixel();
        
        //arm
        liftUpOrDown();
        telemetry.update();
      }
    
  }

  private void pixel(){
    if(gamepad2.right_bumper) intake.setPower(-1);
    else if(gamepad2.left_bumper) intake.setPower(1);
    else intake.setPower(0);
    
    if(gamepad2.a || gamepad2.dpad_up) wrist.setPosition(0);
    if(gamepad2.b) wrist.setPosition(1);
    
    if(gamepad2.x) claw.setPosition(.2);
    if(gamepad2.y || gamepad2.dpad_up) claw.setPosition (0);

    if(gamepad2.dpad_down && armleft.getCurrentPosition() <= 1000){
      wrist.setPosition(1);
      claw.setPosition(.2);
    }

    if(gamepad1.a){
      pullWrist.getController().close();
    } 
    if(gamepad1.b){
      pullWrist.setPosition(0);
    }
    
    if(gamepad1.dpad_down){
            pull.setPower(1);
    }
    else if(gamepad1.dpad_up){
            pull.setPower(-1);
    }
    else pull.setPower(0);
    
    if(gamepad1.right_bumper) launcher.setPosition(.2);
  }
  
  
  
  //INIT CODE
   private void initEverything() {
     
    //Declaration for drivetrain 
    frontleft = hardwareMap.get(DcMotor.class, "frontleft");
    backleft = hardwareMap.get(DcMotor.class, "backleft");
    frontright = hardwareMap.get(DcMotor.class, "frontright");
    backright = hardwareMap.get(DcMotor.class, "backright");
    
    //Declartion for Arms, Intake, and Pull-up bar
   
    intake = hardwareMap.get(DcMotor.class, "intake");
    pull = hardwareMap.get(DcMotor.class, "pull");
    
    //Servos and misc
    launcher = hardwareMap.get(Servo.class, "launcher");
    claw = hardwareMap.get(Servo.class, "claw");
    wrist = hardwareMap.get(Servo.class, "wrist");
    pullWrist = hardwareMap.get(Servo.class, "pullWrist");
    
   //arms & drivetrain
    initArms();
    initTrain();
  }
  
  
  
  
  
  
  /*
  * ARM CODE
  *
  */
  public void initArms(){
    armleft = hardwareMap.get(DcMotor.class, "armleft");
    armright = hardwareMap.get(DcMotor.class, "armright");
    armleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    armright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    // Slides are set back to 0, slides must be down
    goToPosition = 0;
    armleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    armright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    telemetry.addData("Warning", "Spudnik Linear Slides must be all the way down before  'Init' , if not, push them down and 'Init' again.");
    telemetry.update();
    // Keeps the linear slides in sync with each other
    armleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    armright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    armleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    armright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
  }
  
    private void liftUpOrDown() {
    if (gamepad2.dpad_down) {
      // Lift goes up
      armleft.setPower(-1);
      armright.setPower(1);
    } else if (gamepad2.dpad_up) {
      // Lift goes down
      armleft.setPower(0.7);
      armright.setPower(-0.7);
    } else {
      // Lift Brakes, Behavior set for Zero_Power
      if (!gamepad2.a) {
        armleft.setPower(0);
        armright.setPower(0);
      }
    }
    telemetry.addData("Left Lift Postion", armleft.getCurrentPosition());
    telemetry.addData("Right Lift Postion", armright.getCurrentPosition());
    telemetry.update();
  }
  
  
  
  
  /*
  * DRIVETRAIN CODE
  *
  */
  private void drive() {
    float y;
    double x;
    double rx;
    double denominator;

    // Mecanum Drive using left and right sticks
    y = gamepad1.left_stick_y;
    x = -0.5 * gamepad1.left_stick_x;
    // Counteract imperfect strafing
    rx = -gamepad1.right_stick_x * 0.5;
    denominator = JavaUtil.maxOfList(JavaUtil.createListWith(JavaUtil.sumOfList(JavaUtil.createListWith(Math.abs(y), Math.abs(x), Math.abs(rx))), 1));
    frontleft.setPower(((y + x + rx) / denominator) * drivePowerFactor);
    backleft.setPower((((y - x) + rx) / denominator) * drivePowerFactor);
    frontright.setPower((((y - x) - rx) / denominator) * drivePowerFactor);
    backright.setPower((((y + x) - rx) / denominator) * drivePowerFactor * -1);
  }
  public void initTrain(){
     // Reduces the overall power to drive wheels
    drivePowerFactor = 1;
    // Drive motors brake wihen there is zero power
    backleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    backright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    frontleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    frontright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    backleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    backright.setDirection(DcMotor.Direction.REVERSE);
    backright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    frontleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    frontright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    // Mirrored motors are reversed to go the same direction
    frontright.setDirection(DcMotor.Direction.REVERSE);
    backright.setDirection(DcMotor.Direction.REVERSE);
    // Keeps drive-motor rotations in sync  with each other
  }
  
}



