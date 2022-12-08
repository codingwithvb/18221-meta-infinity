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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
//*****Vedanta we called this class JustReadTheInstructions while testing ;)*****
@TeleOp(name="Gamepad", group="Linear Opmode")
//@Disabled
public class Gamepad extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    // private DcMotor leftDrive = null;
    // private DcMotor rightDrive = null;

    DcMotor FrontRight;
    DcMotor FrontLeft;
    DcMotor BackRight;
    DcMotor BackLeft;

    DcMotor leftIntakeMotor;
    DcMotor rightIntakeMotor;
    DcMotor rightSlide;
    DcMotor leftSlide;

    Servo clawServo;
    Servo armServo;

    TouchSensor touchSensor;

    double clawServoPosition = 1.0;
    double armServoPosition = 1.0;

    double horizontal;
    double vertical;
    double pivot;
    double sensitivity;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        FrontLeft  = hardwareMap.dcMotor.get("LeftFront");
        FrontRight = hardwareMap.dcMotor.get("RightFront");
        BackRight = hardwareMap.dcMotor.get("RightBack");
        BackLeft = hardwareMap.dcMotor.get("LeftBack");

        leftIntakeMotor = hardwareMap.dcMotor.get("LeftIntake");
        rightIntakeMotor = hardwareMap.dcMotor.get("RightIntake");

        leftSlide = hardwareMap.dcMotor.get("LeftSlide");
        rightSlide = hardwareMap.dcMotor.get("RightSlide");

        armServo = hardwareMap.servo.get("ArmServo");
        armServo.setPosition(armServoPosition);
        clawServo = hardwareMap.servo.get("ClawServo");
        clawServo.setPosition(clawServoPosition);

        touchSensor = hardwareMap.touchSensor.get("Touch");

        FrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        BackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        leftSlide.setDirection(DcMotorSimple.Direction.REVERSE);

        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            //movement code
            horizontal = gamepad1.left_stick_x;
            pivot = gamepad1.right_stick_x;
            vertical = -gamepad1.left_stick_y;
            sensitivity = gamepad1.left_trigger;
            if (sensitivity > 0) {
                vertical = (float) (vertical * 0.5);
                horizontal = (float) (horizontal * 0.5);
                pivot = (float) (pivot * 0.5);
            }
            FrontRight.setPower(-pivot + (vertical - horizontal));
            BackRight.setPower(-pivot + vertical + horizontal);
            FrontLeft.setPower(pivot + vertical + horizontal);
            BackLeft.setPower(pivot + (vertical - horizontal));

            //if statements are for the slide and the power.
            if (gamepad2.right_bumper) {
                myGoToHeightPOS(100, 1);
            }else if (gamepad2.left_bumper){
                myGoToHeightPOS(-100, 1);
            }
            //if statements are for the servo position whether it be arm or claw.
            /*if (gamepad2.b){
                armServoPosition = 0.0;
                armServo.setPosition(armServoPosition);
            }
            else if (gamepad2.y){
                armServoPosition = 1.0;
                armServo.setPosition(armServoPosition);
            }
            */
            if (gamepad2.a){
                clawServoPosition = 0.0;
                clawServo.setPosition(clawServoPosition);
            }
            else if (gamepad2.x){
                clawServoPosition = 1.0;
                clawServo.setPosition(clawServoPosition);
            }
            //if statements are for intake motor
            if(gamepad1.right_trigger>0){
                rightIntakeMotor.setPower(1.0);
                leftIntakeMotor.setPower(-1.0);
            }else{
                rightIntakeMotor.setPower(0.0);
                leftIntakeMotor.setPower(0.0);
            }
            if(gamepad1.left_trigger>0){
                rightIntakeMotor.setPower(-0);
                leftIntakeMotor.setPower(0);
            }else{
                rightIntakeMotor.setPower(1.0);
                leftIntakeMotor.setPower(-1.0);
            }
            //I am now working with the slide.

            int v_leftSlidePosition = leftSlide.getCurrentPosition();
            telemetry.addData("Left Position", v_leftSlidePosition);
            int v_rightSlidePosition = rightSlide.getCurrentPosition();
            telemetry.addData("Right Position", v_rightSlidePosition);

            //to go up in left slide, lower the value; to go down in left slide, increase the value.
            //to go up in right slide, increase the value; to go down in right slide, decrease the value.

            //macros
            if(gamepad2.dpad_up) {
                myGoToHeightPOS(3450, 1);
            }
            if(gamepad2.dpad_down){
                myGoToHeightPOS(-3450, .5);
            }
            //allows the servos to be controlled by one button per servo, instead of needing 2 per servo
            if(gamepad2.x && clawServo.getPosition()==0.0)
                clawServo.setPosition(1.0);
            else if (gamepad2.x && clawServo.getPosition()==1.0)
                clawServo.setPosition(0.0);
            if(gamepad2.y && armServo.getPosition()==1.0)
                armServo.setPosition(0.25);
            else if (gamepad2.y && armServo.getPosition()==0.25)
                armServo.setPosition(1.0);

            /*if(gamepad2.x){
                clawServo.setPosition(1.0);
            }
            if(gamepad2.y){
                clawServo.setPosition(0.0);
            }
            if(gamepad2.a){
                armServo.setPosition(0.25);
            }
            if(gamepad2.b){
                armServo.setPosition(1.0);
            } */


            if(touchSensor.isPressed()){
                leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                telemetry.addData("Touch Sensor Pressed", v_leftSlidePosition);
                telemetry.update();
            }else {
                telemetry.addData("Not Pressed", v_leftSlidePosition);
                telemetry.update();
            }
        }
    }
    public void myGoToHeightPOS(int slidePOS, double motorPower) {
        //to find slide position and motor position
        telemetry.addData("slidePOS", slidePOS);
        telemetry.update();
        telemetry.addData("motorPower", motorPower);
        telemetry.update();
        //base encoder code
        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSlide.setTargetPosition((slidePOS-50));
        rightSlide.setTargetPosition(slidePOS);
        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftSlide.setPower(motorPower);
        rightSlide.setPower(motorPower);

        //if(slidePOS>=3500 || slidePOS<=-3500) {
            //leftSlide.setPower(0.0);
            //telemetry.addData("Maximum Reached", slidePOS);
            //telemetry.update();
       // }
    }
}
