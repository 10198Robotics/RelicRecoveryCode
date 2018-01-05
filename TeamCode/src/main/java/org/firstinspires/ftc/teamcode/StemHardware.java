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

import android.hardware.Sensor;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */
public class StemHardware
{
    /* Public OpMode members. */
    public DcMotor  FR          = null;
    public DcMotor  FL          = null;
    public DcMotor  BR          = null;
    public DcMotor  BL          = null;
    public DcMotor  lift     = null;
    public DcMotor  RelicArm = null;
    public Servo    RA1      = null;
    public Servo    RA2      = null;
   public Servo    Claw1     = null;
    public Servo    Claw2    = null;
     public Servo    elbow    = null;
    public Servo    wrist   = null;


    //OpticalDistanceSensor ODS;


  //  public static final double OPEN      =  .7 ;
    //public static final double CLOSE   =  0.25 ;
    //public static final double MIDDLE  =  0.4;


    /* local OpMode members. */

    HardwareMap hwMap = null;
    /* Constructor */


    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map

        hwMap = ahwMap;


        // Define and Initialize Motors
        FR  = hwMap.get(DcMotor.class, "FR");
        FL = hwMap.get(DcMotor.class, "FL");
        BR  = hwMap.get(DcMotor.class, "BR");
        BL = hwMap.get(DcMotor.class, "BL");
        lift   = hwMap.get(DcMotor.class, "lift");
        RelicArm = hwMap.get(DcMotor.class,  "Relic_Arm");

        FR.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        FL.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        BR.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        BL.setDirection(DcMotor.Direction.FORWARD);

        // Set all motors to zero power
        FR.setPower(0);
        FL.setPower(0);
        BR.setPower(0);
        BL.setPower(0);
        lift.setPower(0);
        //RelicArm.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RelicArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Define and initialize ALL installed servos.
        wrist = hwMap.get(Servo.class,  "wrist");

        elbow = hwMap.get(Servo.class,  "elbow");
        Claw1  = hwMap.get(Servo.class, "claw1");
        Claw2  = hwMap.get(Servo.class, "claw2");
        RA1 = hwMap.get(Servo.class, "RA1");
        RA2 = hwMap.get(Servo.class, "RA2");

        elbow.setPosition(0);
        RA1.setPosition(.1);
        RA2.setPosition(.1);
        Claw1.setPosition(.3);
        Claw2.setPosition(.6);
        wrist.setPosition(.6);



    }
}
