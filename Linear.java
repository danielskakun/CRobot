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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.ServoControllerEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.Servo;
/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="LinearOp", group="Linear Opmode")

public class Linear extends LinearOpMode {

    private Elapsed1Time runtime = new ElapsedTime();
    private DcMotor LF;
    private DcMotor RF;
    private DcMotor LB;
    private DcMotor RB;
    private Servo FirstServo;
    DcMotor Beam;
    Servo ServoHandle;
    Servo ServoHandle1;
    Servo ServoHandleTopLeft;
    Servo ServoHandleTopRight;

    double increment=0;
    Servo Pivot; 
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        Beam=hardwareMap.get(DcMotor.class,"beam");
        LF  = hardwareMap.get(DcMotor.class, "leftfront");
        RF = hardwareMap.get(DcMotor.class, "rightfront");
        LB  = hardwareMap.get(DcMotor.class, "leftback");
        RB = hardwareMap.get(DcMotor.class, "rightback");
        ServoHandle = hardwareMap.get(Servo.class, "ServoHandle");
        ServoHandle1= hardwareMap.get(Servo.class, "ServoHandle1");
        Pivot=hardwareMap.get(Servo.class, "pivot");
        ServoHandleTopLeft= hardwareMap.get(Servo.class, "3");
        ServoHandleTopRight= hardwareMap.get(Servo.class, "4");
        


        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

    while(opModeIsActive()) {



        
       if (gamepad1.right_bumper){
      ServoHandle.setPosition(1);
      ServoHandle1.setPosition(0);
      ServoHandleTopLeft.setPosition(1);
      ServoHandleTopRight.setPosition(0);

      }
      else if (gamepad1.left_bumper){
      ServoHandle.setPosition(.3);
      ServoHandle1.setPosition(.7);    
      ServoHandleTopLeft.setPosition(.3);
      ServoHandleTopRight.setPosition(.7);
     }
     else if (gamepad1.a){
      ServoHandle.setPosition(.7);
      ServoHandle1.setPosition(.2);    
      ServoHandleTopLeft.setPosition(.7);
      ServoHandleTopRight.setPosition(.2);
     }
       else if  (gamepad1.x) {
Beam.setPower(-.4);
            }
       else if (gamepad1.y) {
 Beam.setPower(0);                 
            }
       else if (gamepad1.b) {
Beam.setPower(.4);
            }
        else if (gamepad1.dpad_up ){
            Pivot.setPosition(1);
     

        }
            // Setup a variable for each drive wheel to save power level for telemetry
        float LBSpeed=  gamepad1.left_stick_y; 
        float LFSpeed=  gamepad1.left_stick_y;
        float RBSpeed= -gamepad1.right_stick_y; 
        float RFSpeed= -gamepad1.right_stick_y;

       // LBSpeed= Range.clip(LBSpeed, -1, 1);
        //LFSpeed= Range.clip(LFSpeed, -1, 1);
        //RBSpeed= Range.clip(RBSpeed, 1, -1);
        //RFSpeed= Range.clip(RFSpeed, 1, -1);


            // Choose to drive using either Tank Mode, or POV Mode
            // Comment out the method that's not used.  The default below is POV.

            // POV Mode uses left stick to go forward, and right stick to turn.

            LF.setPower(LFSpeed);
            LB.setPower(LBSpeed);
            RF.setPower(RFSpeed);
            RB.setPower(RBSpeed);
            
            
            

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
    
        }
    }
    
}




