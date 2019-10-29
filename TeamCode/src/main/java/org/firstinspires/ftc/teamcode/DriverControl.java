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
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.HardwareRobot;

/**
 * Demonstrates empty OpMode
 */
@TeleOp(name = "Concept: NullOp", group = "Concept")
@Disabled
public class DriverControl extends OpMode {
  HardwareRobot robot = new HardwareRobot();

  private ElapsedTime runtime = new ElapsedTime();

  @Override
  public void init() {
    telemetry.addData("Status", "Initialized");

    robot.init(hardwareMap);
  }
  /*
     * Code to run when the op mode is first enabled goes here
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
     */


  @Override
  public void init_loop() {
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
    telemetry.addData("Status", "Run Time: " + runtime.toString());
  }
  boolean RB1isPressed  = gamepad1.right_bumper;
  boolean LB1isPressed  = gamepad1.left_bumper;
  boolean B1isPressed   = gamepad1.b;
  boolean A1isPressed   = gamepad1.a;
  boolean Y1isPressed   = gamepad1.y;
  boolean X1isPressed   = gamepad1.x;
  boolean dPadUp1       = gamepad1.dpad_up;
  boolean dPadDown1     = gamepad1.dpad_down;
  boolean dPadLeft1     = gamepad1.dpad_left;
  boolean dPadRight1    = gamepad1.dpad_right;
  boolean R3isPressed1  = gamepad1.right_stick_button;
  boolean L3isPressed1  = gamepad1.left_stick_button;

  double RT1            = gamepad1.right_trigger;
  double LT1            = gamepad1.left_trigger;
  double leftStickY1    = gamepad1.left_stick_y;
  double leftStickX1    = gamepad1.left_stick_x;
  double rightStickY1   = gamepad1.right_stick_y;
  double rightStickX1   = gamepad1.right_stick_x;

  //gamepad2
  boolean RB2isPressed = gamepad2.right_bumper;
  boolean LB2isPressed  = gamepad2.left_bumper;
  boolean B2isPressed   = gamepad2.b;
  boolean A2isPressed   = gamepad2.a;
  boolean Y2isPressed   = gamepad2.y;
  boolean X2isPressed   = gamepad2.x;
  boolean dPadUp2       = gamepad2.dpad_up;
  boolean dPadDown2     = gamepad2.dpad_down;
  boolean dPadLeft2     = gamepad2.dpad_left;
  boolean dPadRight2    = gamepad2.dpad_right;
  boolean R3isPressed2  = gamepad2.right_stick_button;
  boolean L3isPressed2  = gamepad2.left_stick_button;

  double RT2            = gamepad2.right_trigger;
  double LT2            = gamepad2.left_trigger;
  double leftStickY2    = gamepad2.left_stick_y;
  double leftStickX2    = gamepad2.left_stick_x;
  double rightStickY2   = gamepad2.right_stick_y;
  double rightStickX2   = gamepad2.right_stick_x;

}
