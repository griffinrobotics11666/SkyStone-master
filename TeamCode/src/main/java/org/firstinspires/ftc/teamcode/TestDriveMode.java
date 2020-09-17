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
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Test Drive Mode", group="Iterative Opmode")
//@Disabled
public class TestDriveMode extends OpMode
{

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null; //define leftDrive
    private DcMotor rightDrive = null; //define rightDrive

    Servo rightArmServo; //define armServo
    Servo leftArmServo;
    static final double INCREMENT   = 0.003;     // amount to slew servo each CYCLE_MS cycle
    static final double MAX_POS     =  1.0;     // Maximum rotational position
    static final double MIN_POS     =  0.0;     // Minimum rotational position


    double  rightServoPosition = (MAX_POS - MIN_POS) / 2; // Start at halfway position
    double  leftServoPosition = (MAX_POS - MIN_POS) / 2; // Start at halfway position

    boolean debug = false;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        rightArmServo = hardwareMap.get(Servo.class, "right_arm_servo");
        leftArmServo = hardwareMap.get(Servo.class, "left_arm_servo");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);
        rightArmServo.setPosition(rightServoPosition);
        leftArmServo.setPosition(leftServoPosition);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");


    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        // Setup a variable for each drive wheel to save power level for telemetry
        double leftPower;
        double rightPower;






            if (gamepad1.dpad_right){   //rotates the servo right
                rightServoPosition += INCREMENT; //increases position
                leftServoPosition -= INCREMENT;
                if (rightServoPosition >= MAX_POS || leftServoPosition >= MAX_POS ) {
                    rightServoPosition = MAX_POS;
                    leftServoPosition = MAX_POS;
                }
                rightArmServo.setPosition(rightServoPosition);
                leftArmServo.setPosition(leftServoPosition);


            }

            if (gamepad1.dpad_left) {
                rightServoPosition -= INCREMENT; //increases position
                leftServoPosition += INCREMENT;
                if (rightServoPosition <= MIN_POS || leftServoPosition <= MIN_POS) {
                    rightServoPosition = MIN_POS;
                    leftServoPosition = MIN_POS;
                }
                rightArmServo.setPosition(rightServoPosition);
                leftArmServo.setPosition(leftServoPosition);

            }






        double turn = -gamepad1.right_stick_x;
        double drive  =  gamepad1.left_stick_y;

        leftPower    = Range.clip(drive + turn, -0.5, 0.5) ;
        rightPower   = Range.clip(drive - turn, -0.5, 0.5) ;


        // Send calculated power to wheels
        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
        telemetry.addData("Left servo Position", "%5.2f", leftServoPosition);
        telemetry.addData("Right servo Position", "%5.2f", rightServoPosition);
        telemetry.update();

        if(gamepad1.a) {
            debug = true;
        }

        while(debug) {
            if(gamepad1.b) {
                leftServoPosition += INCREMENT;
                leftArmServo.setPosition(leftServoPosition);
                leftArmServo.getPosition();
                gamepad1.b = false;
            }
            if(gamepad1.a) {
                leftServoPosition -= INCREMENT;
                leftArmServo.setPosition(leftServoPosition);
                leftArmServo.getPosition();
                gamepad1.a = false;
            }
            if(gamepad1.y) {
                rightServoPosition += INCREMENT;
                rightArmServo.setPosition(rightServoPosition);
                rightArmServo.getPosition();
                gamepad1.y = false;
            }
            if(gamepad1.x) {
                rightServoPosition -= INCREMENT;
                rightArmServo.setPosition(rightServoPosition);
                rightArmServo.getPosition();
                gamepad1.x = false;
            }

        }

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
