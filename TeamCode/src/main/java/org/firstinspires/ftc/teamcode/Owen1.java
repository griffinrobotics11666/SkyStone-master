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
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Demonstrates empty OpMode
 */
@TeleOp(name = "Concept: NullOp", group = "Concept")
//@Disabled
public class Owen1 extends OpMode {

  private ElapsedTime runtime = new ElapsedTime();
  private DcMotor leftDrive = null;
  private DcMotor rightDrive = null;
  private DcMotor armMotor = null;
  static final double INCREMENT = 0.001;
  static final double MAX_POS_S1 = 1.0;
  static final double MIN_POS_S1 = 0.0;
  static final double MAX_POS_S2 = 1.0;
  static final double MIN_POS_S2 = 0.0;
  static final double MAX_POS_S3 = 1.0;
  static final double MIN_POS_S3 = 0.0;

  boolean leftClawOpen = false;
  boolean rightClawOpen = false;
  boolean wristUp = true;

  double leftClawOpenValue = 0.72;
  double leftClawClosedValue = 0.92;
  double rightClawOpenValue = 0.6;
  double rightClawClosedValue = 0.77;
  double wristUpValue = 1;
  double wristDownValue = 0;

  Servo wristServo;
  Servo rightClawServo;
  Servo leftClawServo;
  double s1Pos= ((MAX_POS_S1 - MIN_POS_S1) / 2);
  double s2Pos = ((MAX_POS_S2 - MIN_POS_S2) / 2);
  double s3Pos = ((MAX_POS_S3 - MIN_POS_S3) / 2);
  boolean RBisPressed;//  = gamepad1.right_bumper;
  boolean LBisPressed;//  = gamepad1.left_bumper;
  boolean BisPressed;
  boolean AisPressed;
  boolean YisPressed;
  boolean XisPressed;
  double leftPower;
  double rightPower;
  double drive;
  double turn;
  double armPower;

  @Override
  public void init() {
    telemetry.addData("Status", "Initialized");

    leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
    rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
    armMotor = hardwareMap.get(DcMotor.class, "arm_motor");
    wristServo = hardwareMap.get(Servo.class, "arm_rotation");
    rightClawServo = hardwareMap.get(Servo.class, "right_claw_movement");
    leftClawServo = hardwareMap.get(Servo.class, "left_claw_movement");
    leftDrive.setDirection(DcMotor.Direction.FORWARD);
    rightDrive.setDirection(DcMotor.Direction.REVERSE);
    armMotor.setDirection(DcMotor.Direction.FORWARD);
    telemetry.addData("Status", "Initialized");

  }
  /*
   * Code to run when the op mode is first enabled goes here
   * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
   */

  public void runOpMode() {

    telemetry.update();


  }


  @Override
  public void init_loop() {
    wristServo.setPosition(s1Pos);
    rightClawServo.setPosition(s2Pos);
    leftClawServo.setPosition(s3Pos);
  }

  /*
   * This method will be called ONCE when start is pressed
   * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#loop()
   */
  @Override
  public void start() {
    runtime.reset();
  }

  /*
   * This method will be called repeatedly in a loop
   * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#loop()
   */
  @Override
  public void loop() {

    armPower = gamepad1.right_stick_y;
    drive = -gamepad1.left_stick_y;
    turn = gamepad1.left_stick_x;
    RBisPressed = gamepad1.right_bumper;
    LBisPressed = gamepad1.left_bumper;
    BisPressed = gamepad1.b;
    AisPressed = gamepad1.a;
    YisPressed = gamepad1.y;
    XisPressed = gamepad1.x;

    leftPower = Range.clip(drive + turn, -1.0, 1.0);
    rightPower = Range.clip(drive - turn, -1.0, 1.0);
    armPower = Range.clip(armPower, -1.0, 1.0);

    leftDrive.setPower(leftPower);
    rightDrive.setPower(rightPower);
    armMotor.setPower(armPower);

    if (AisPressed) {
      s3Pos = leftClawOpenValue;
      s2Pos = rightClawClosedValue;
    }
    if (BisPressed){
      s3Pos = leftClawClosedValue;
      s2Pos = rightClawOpenValue;
    }
    //if (YisPressed){}

    //if (XisPressed){}

     if (LBisPressed){
       s1Pos = wristDownValue;}
     if (RBisPressed){
       s1Pos = wristUpValue;}




//    if (RBisPressed) {
//      s1Pos += INCREMENT;
//    } else if (LBisPressed) {
//      s1Pos -= INCREMENT;
//    }
//
//    if(BisPressed) {
//     s2Pos += INCREMENT; }
//    else if (AisPressed) {
//      s2Pos -= INCREMENT; }
//
//    if(YisPressed) {
//      s3Pos += INCREMENT; }
//    else if(XisPressed) {
//      s3Pos -= INCREMENT; }

    leftClawServo.setPosition(s3Pos);
    rightClawServo.setPosition(s2Pos);
    wristServo.setPosition(s1Pos);

    telemetry.addData("Status", "Run Time: " + runtime.toString());
    telemetry.addData("Motors", "left (%.2f), right (%.2f), arm (%.2f)", leftPower, rightPower, armPower);
    telemetry.addData("Servos", "left_claw (%.2f), right_claw (%.2f), wrist (%.2f)", s3Pos, s2Pos, s1Pos);

  }
  public void stop() {}
}

