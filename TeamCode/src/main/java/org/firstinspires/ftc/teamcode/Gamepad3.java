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
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


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
@TeleOp(name="Gamepad3.0", group="Linear Opmode")
//@Disabled
public class Gamepad3 extends LinearOpMode {

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

    NormalizedColorSensor colorSensor;
    TouchSensor touchSensor;

    double horizontal;
    double vertical;
    double pivot;
    double sensitivity;

    double gc_armOut = 0;         // Arm position claw outside
    double gc_armIn = 0.6;         // Arm position claw inside
    double gc_clawOpen = 0.5;      // Claw Open
    double gc_clawClosed = 0.65;     // Claw Closed

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
        armServo.setPosition(gc_armIn);           // Arm Out = 0.1
        clawServo = hardwareMap.servo.get("ClawServo");
        clawServo.setPosition(gc_clawClosed);     // Claw Closed = 1.0

        touchSensor = hardwareMap.touchSensor.get("Touch");
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");

        if (colorSensor instanceof SwitchableLight) {
            ((SwitchableLight)colorSensor).enableLight(true);
        }

        FrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        BackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        leftSlide.setDirection(DcMotorSimple.Direction.REVERSE);

        leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();
        runtime.reset();

        // slides go down half power till hits Touch Sensor or interrupted by driver Gamepad1 A
        /*while(!touchSensor.isPressed()){
            telemetry.addData("Digital Touch", "Is Not Pressed - moving down");
            telemetry.update();
            if(gamepad2.left_bumper){
                leftSlide.setPower(-0.5);
                rightSlide.setPower(-0.5);
            }
            else if(gamepad2.right_bumper){
                leftSlide.setPower(0.5);
                rightSlide.setPower(0.5);
            }
            else {
                leftSlide.setPower(0);
                rightSlide.setPower(0);
            }
            if(gamepad2.x){
                armServo.setPosition(gc_armOut);
            }
            if(gamepad2.y){
                armServo.setPosition(gc_armIn);
            }
            if(gamepad1.a)
                break;      // Stop the downward movement if something goes wrong
        }*/
        // Stop the slide down and reset encoders to zero position
        leftSlide.setPower(0);
        rightSlide.setPower(0);
        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // Stop slide motors if the touch sensor is pressed
            // if the digital channel returns true it's HIGH and the button is unpressed.
            if (touchSensor.isPressed() || gamepad1.a) {
                telemetry.addData("Digital Touch", "Is Pressed - reset encoder");
                leftSlide.setPower(0);
                rightSlide.setPower(0);
                leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            } else {
                telemetry.addData("Digital Touch", "Is Not Pressed");
            }
            //check for cone within 12 cm of color sensor
            //close claw and prime slides if ready
            telemetry.addData("Distance",((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM));
            telemetry.update();
            if(((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM)<12){
                AutoMoveClaw();
            }
            //movement code for the robot
            horizontal = gamepad1.left_stick_x;
            pivot = gamepad1.right_stick_x;
            vertical = -gamepad1.left_stick_y;
            sensitivity = gamepad1.left_trigger;
            if (sensitivity > 0) {
                vertical = (float) (vertical * 0.5);
                horizontal = (float) (horizontal * 0.5);
                pivot = (float) (pivot * 0.5);
            }
            FrontRight.setPower(0.7*(-pivot + (vertical - horizontal)));
            BackRight.setPower(0.7*(-pivot + vertical + horizontal));
            FrontLeft.setPower(0.7*(pivot + vertical + horizontal));
            BackLeft.setPower(0.7*(pivot + (vertical - horizontal)));

            //if statements are for the servo position whether it be arm or claw.
            if(gamepad2.b){
                clawServo.setPosition(gc_clawClosed);
            }
            if(gamepad2.a){
                clawServo.setPosition(gc_clawOpen);
            }
            if(gamepad2.x){
                armServo.setPosition(0.3);
                //armServo.setPosition(gc_armOut);
            }
            if(gamepad2.y){
                armServo.setPosition(gc_armIn);
            }

            //micro adjustments
            if(gamepad2.right_bumper){            // Move up 300 - GP2 - Right Bumper
                myGoToHeightPOS(leftSlide.getCurrentPosition()+300,0.5);
            }
            if(gamepad2.left_bumper){            // Move down 300 - GP2 - Right Bumper
                myGoToHeightPOS(leftSlide.getCurrentPosition()-300,0.5);
            }

            int v_leftSlidePosition = leftSlide.getCurrentPosition();
            telemetry.addData("Left Position", v_leftSlidePosition);
            int v_rightSlidePosition = rightSlide.getCurrentPosition();
            telemetry.addData("Right Position", v_rightSlidePosition);
            telemetry.update();

            //to go up in left slide, lower the value; to go down in left slide, increase the value.
            //to go up in right slide, increase the value; to go down in right slide, decrease the value.

            //macros

            // GP2 DPAD Right - linear slide goes up, Arm Out for high junction - 3850
            if(gamepad2.dpad_right) {
            }

            // GP2 DPAD Left - linear slide goes up, Arm Out for Low junction - 1700
            if(gamepad2.dpad_left) {
            }

            //  GP2 DPAD Up - linear slide goes up, Arm Out for Medium junction - 2700
            if(gamepad2.dpad_up) {
            }

            //  GP2 DPAD Down - Claw down, Arm In for picking intake cone - 0
            if(gamepad2.dpad_down){
                myGoToHeightPOS(300, 1);
                while(leftSlide.isBusy()||rightSlide.isBusy()){
                    clawServo.setPosition(gc_clawClosed);
                    FrontRight.setPower(0.7*(-pivot + (vertical - horizontal)));
                    BackRight.setPower(0.7*(-pivot + vertical + horizontal));
                    FrontLeft.setPower(0.7*(pivot + vertical + horizontal));
                    BackLeft.setPower(0.7*(pivot + (vertical - horizontal)));
                }
                armServo.setPosition(gc_armIn);
                clawServo.setPosition(gc_clawClosed);
                sleep(600);
                myGoToHeightPOS(0, 1);
            }
        }
    }
    public void myGoToHeightPOS(int slidePOS, double motorPower) {
        //to find slide position and motor position
        telemetry.addData("leftSlide", leftSlide.getCurrentPosition());
        telemetry.addData("rightSlide", rightSlide.getCurrentPosition());
        telemetry.addData("motorPower", motorPower);
        telemetry.update();
        //base encoder code
        //leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSlide.setTargetPosition(slidePOS);
        rightSlide.setTargetPosition(slidePOS);
        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftSlide.setPower(motorPower);
        rightSlide.setPower(motorPower);
        
    }

    public void AutoMoveClaw(){
        clawServo.setPosition(gc_clawClosed);
        sleep(500);
        armServo.setPosition(0.3);
    }

}
