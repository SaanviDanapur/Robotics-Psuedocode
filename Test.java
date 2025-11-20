package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "BlockTest2 (Blocks to Java)")
public class BlockTest2 extends LinearOpMode {

  private DcMotor backRightMotor;

  /**
   * Describe this function...
   */
  private void motor() {
    backRightMotor.setDirection(DcMotor.Direction.REVERSE);
  }

  /**
   * This sample contains the bare minimum Blocks for any regular OpMode. The 3 blue
   * Comment Blocks show where to place Initialization code (runs once, after touching the
   * DS INIT button, and before touching the DS Start arrow), Run code (runs once, after
   * touching Start), and Loop code (runs repeatedly while the OpMode is active, namely not
   * Stopped).
   */
  @Override
  public void runOpMode() {
    backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");

    // Put initialization blocks here.
    waitForStart();
    if (opModeIsActive()) {
      // Put run blocks here.
      while (opModeIsActive()) {
        // Put loop blocks here.
        if (gamepad1.circle) {
          telemetry.addLine("circle");
          backRightMotor.setPower(0.2);
        }
        if (gamepad1.cross) {
          backRightMotor.setPower(0);
          telemetry.addLine("cross");
        }
        telemetry.update();
      }
    }
  }
}
