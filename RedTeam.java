/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import java.util.Map;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file illustrates the concept of driving a path based on encoder counts.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forwards, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backwards for 24 inches
 *   - Stop and close the claw.
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This methods assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Red Team", group="Pushbot")

public class RedTeam extends LinearOpMode {


    public ElapsedTime     runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 1300 ;    // eg: TETRIX Motor Encoder
   // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV) /
                                                      (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;

    private DcMotor LF;
    private DcMotor RF;
    private DcMotor LB;
    private DcMotor RB;
    private Servo FirstServo;
    private Servo Pivot;
    ColorSensor Color;
    DistanceSensor Distance;
      Servo ServoHandle;
    Servo ServoHandle1;
    Servo ServoHandleTopLeft;
    Servo ServoHandleTopRight;
     Servo Arm;
     Servo Turn;
    DcMotor Beam;

    @Override
    public void runOpMode() throws InterruptedException {
        Pivot=hardwareMap.get(Servo.class, "pivot");
    //    Distance=hardwareMap.get(DistanceSensor.class, "sensor_color_distance");
        Color=hardwareMap.get(ColorSensor.class, "sensor_color_distance");
         LF  = hardwareMap.get(DcMotor.class, "leftfront");
        RF = hardwareMap.get(DcMotor.class, "rightfront");
        LB  = hardwareMap.get(DcMotor.class, "leftback");
        RB = hardwareMap.get(DcMotor.class, "rightback");
     //   Arm= hardwareMap.get(Servo.class, "Arm");
        LF.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        RF.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        LB.setDirection(DcMotor.Direction.REVERSE);
        RB.setDirection(DcMotor.Direction.FORWARD);
        ServoHandle = hardwareMap.get(Servo.class, "ServoHandle");
        ServoHandle1= hardwareMap.get(Servo.class, "ServoHandle1");
        ServoHandleTopLeft= hardwareMap.get(Servo.class, "3");
        ServoHandleTopRight= hardwareMap.get(Servo.class, "4");
        Beam=hardwareMap.get(DcMotor.class,"beam");
        Turn= hardwareMap.get(Servo.class,"turn");

waitForStart();

        resetStartTime();
        int blueLeft;
        
        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();


        telemetry.addData("Path0",  "Starting at %7d :%7d",
                          LF.getCurrentPosition(),
                         RF.getCurrentPosition());
        telemetry.update();
        resetEncoder();
           ServoHandle.setPosition(1);
      ServoHandle1.setPosition(0);
      ServoHandleTopLeft.setPosition(1);
      ServoHandleTopRight.setPosition(0);
       resetStartTime();
      
    Turn.setPosition(.55);
      Pivot.setPosition(0);
      sleep(1000);
        blueLeft= Color.blue();
        if(blueLeft>=23){
        Turn.setPosition(.2);
        sleep(1000);
        Turn.setPosition(.4);
        sleep(1000);
        Pivot.setPosition(1);

        }
        else {
       Turn.setPosition(.8);
        sleep(1000);
        Turn.setPosition(.6);
        sleep(1000);
      Pivot.setPosition(1);

        }
   
 Beam.setPower(0);
       



        telemetry.addData("Path", "Complete");
        telemetry.update();
        
    }
    
    
    public void resetEncoder(){
         LF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       
        LF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeout) {
        int newLFTarget;
        int newRFTarget;
       

        if (opModeIsActive()) {

            
            newLFTarget = LF.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRFTarget = RF.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
        
            LF.setTargetPosition(newLFTarget);
            RF.setTargetPosition(newRFTarget);
           

             LF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      
            


            // reset the timeout time and start motion.
            runtime.reset();
            LF.setPower(Math.abs(speed));
            RF.setPower(Math.abs(speed));
           
        

           
            while (opModeIsActive() &&
                   (runtime.seconds() < timeout) &&
                   (LF.isBusy() && RF.isBusy() )) {

                telemetry.addData("Path1",  "Running to %7d :%7d", newLFTarget,  newRFTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",LF.getCurrentPosition(), RF.getCurrentPosition());
                                        
                telemetry.update();
            }

            LF.setPower(0);
            RF.setPower(0);
            LB.setPower(0);
            RB.setPower(0);
            

            LF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            RF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                

             sleep(250);   // optional pause after each move
        }
    }
}
