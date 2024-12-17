package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "USETHISONE (Blocks to Java)")
public class USETHISONE extends LinearOpMode {

  private DcMotor br;
  private DcMotor fr;
  private DcMotor arm;
  private DcMotor hang;
  private CRServo outakeServo;
  private DcMotor slide;
  private DcMotor bl;
  private DcMotor fl;

  /**
   * This sample contains the bare minimum Blocks for any regular OpMode. The 3 blue
   * Comment Blocks show where to place Initialization code (runs once, after touching the
   * DS INIT button, and before touching the DS Start arrow), Run code (runs once, after
   * touching Start), and Loop code (runs repeatedly while the OpMode is active, namely not
   * Stopped).
   */
  @Override
  public void runOpMode() {
    br = hardwareMap.get(DcMotor.class, "br");
    fr = hardwareMap.get(DcMotor.class, "fr");
    arm = hardwareMap.get(DcMotor.class, "arm");
    hang = hardwareMap.get(DcMotor.class, "hang");
    outakeServo = hardwareMap.get(CRServo.class, "outakeServo");
    slide = hardwareMap.get(DcMotor.class, "slide");
    bl = hardwareMap.get(DcMotor.class, "bl");
    fl = hardwareMap.get(DcMotor.class, "fl");

    br.setDirection(DcMotor.Direction.REVERSE);
    fr.setDirection(DcMotor.Direction.REVERSE);
    waitForStart();
    DriveWithEncoder(0.7, -150, -150, -150, -150);
    runArm(0.4, 0.7);
    runSlide(0.7, 3.25);
    DriveWithEncoder(0.7, -1500, 1500, 1500, -1500);
    RightPivot(300, 300);
    DriveWithEncoder(0.7, 100, 100, 100, 100);
    runSlide(0.7, 0.35);
    runOuttake(5);
    runSlide(0.7, 0.1);
    DriveWithEncoder(0.7, -300, -300, -300, -300);
    arm.setPower(-0.5);
    sleep(2000);
    arm.setPower(0);
    arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    runSlide(-0.7, 2.9);
    DriveWithEncoder(0.7, -200, -200, -400, -400);
    DriveWithEncoder(0.7, 1500, -1500, -1500, 1500);
    DriveWithEncoder(0.7, -200, -200, 200, 200);
    DriveWithEncoder(0.7, -1100, -1100, -1100, -1100);
    DriveWithEncoder(0.7, -1000, -1000, 1000, 1000);
    hang.setPower(-0.5);
    sleep(800);
    hang.setPower(0);
    hang.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    DriveWithEncoder(0.8, -1200, -1200, -1200, -1200);
  }

  /**
   * Describe this function...
   */
  private void runOuttake(int secRun) {
    outakeServo.setPower(-1);
    sleep(1000 * secRun);
    outakeServo.setPower(0);
  }

  /**
   * Describe this function...
   */
  private void runSlide(double slideSpeed, double SecRise) {
    slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    slide.setPower(-1 * slideSpeed);
    sleep((long) (1000 * SecRise));
    while (opModeIsActive() && slide.isBusy()) {
      idle();
    }
    slide.setPower(0);
  }

  /**
   * Describe this function...
   */
  private void runArm(double slideSpeed, double SecRise) {
    arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    arm.setPower(-1 * slideSpeed);
    sleep((long) (1000 * SecRise));
    while (opModeIsActive() && arm.isBusy()) {
      idle();
    }
    arm.setPower(0);
  }

  /**
   * Describe this function...
   */
  private void DriveWithEncoder(double speed, int frontRight, int backRight, int frontLeft, int backLeft) {
    br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    br.setTargetPosition(backRight);
    bl.setTargetPosition(backLeft);
    fr.setTargetPosition(frontRight);
    fl.setTargetPosition(frontLeft);
    fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    br.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    fr.setPower(speed);
    fl.setPower(speed);
    bl.setPower(speed);
    br.setPower(speed);
    while (opModeIsActive() && fr.isBusy() && fl.isBusy() && br.isBusy() && bl.isBusy()) {
      idle();
    }
  }

  /**
   * Describe this function...
   */
  private void RightPivot(int blPivot, int frPivot) {
    bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    bl.setTargetPosition(-1 * blPivot);
    fr.setTargetPosition(frPivot);
    bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    bl.setPower(0.4);
    fr.setPower(0.4);
    while (opModeIsActive() && bl.isBusy() && fr.isBusy()) {
      idle();
    }
  }
}
