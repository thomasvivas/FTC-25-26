package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
// import com.qualcomm.robotcore.hardware.Servo;
// import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
// import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
// import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
// import org.firstinspires.ftc.vision.apriltag;
// import org.firstinspires.ftc.vision.VisionPortal;
// import org.firstinspires.ftc.vision;


// FTC Team 251 represented by Cliffside Park High School
// FTC Decode challenge season 2025 - 2026
// Code authored by Thomas Vivas
// Not one lick of AI to be found in these lines

@TeleOp
public class ManualControl2526 extends LinearOpMode {
  private DcMotor driveFrontLeft;
  private DcMotor driveFrontRight;
  private DcMotor driveBackLeft;
  private DcMotor driveBackRight;

  private DcMotor shootFrontLeft;
  private DcMotor shootFrontRight;
  private DcMotor shootBackLeft;
  private DcMotor shootBackRight;

  // Servo artifactStopper;

  // AprilTagProcessor aprilTagProcessor;
  // VisionPortal visionPortal;

  private BNO055IMU imu;
  // Robot State
  private double currentSpeed;
  private long lastSpeedChangeMillis;

  /////////////////////
  // Core Functions ///
  /////////////////////

  @Override
  public void runOpMode() {
    initializeHardware();
    initializeRobotState();

    telemetry.addData("Status", "Initialized");
    telemetry.update();

    waitForStart();
    while (opModeIsActive()) {
      handleSpeedControl();
      handleMovement();
      handleShooting();
      updateTelemetry();
    }
  }

  // Handles everything connected to the Control Hub and Expansion Hub
  private void initializeHardware() {
    driveFrontLeft = hardwareMap.get(DcMotor.class, "driveFrontLeft");
    driveFrontRight = hardwareMap.get(DcMotor.class, "driveFrontRight");
    driveBackLeft = hardwareMap.get(DcMotor.class, "driveBackLeft");
    driveBackRight = hardwareMap.get(DcMotor.class, "driveBackRight");

    shootFrontLeft = hardwareMap.get(DcMotor.class, "shootFrontLeft");
    shootFrontRight = hardwareMap.get(DcMotor.class, "shootFrontRight");
    shootBackLeft = hardwareMap.get(DcMotor.class, "shootBackLeft");
    shootBackRight = hardwareMap.get(DcMotor.class, "shootBackRight");

    // artifactStopper = hardwareMap.get(Servo.class, "artifactStopper");
    // aprilTagProcessor = AprilTagProcessor.easyCreateWithDefaults();
    // visionPortal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "Webcam 1"), aprilTagProcessor);
  }

  private void initializeRobotState() {
    final double INITIAL_SPEED = 0.6;

    currentSpeed = INITIAL_SPEED;
    lastSpeedChangeMillis = System.currentTimeMillis();

    driveFrontLeft.setDirection(DcMotor.Direction.FORWARD);
    driveFrontRight.setDirection(DcMotor.Direction.REVERSE);
    driveBackLeft.setDirection(DcMotor.Direction.FORWARD);
    driveBackRight.setDirection(DcMotor.Direction.REVERSE);

    shootFrontLeft.setDirection(DcMotor.Direction.FORWARD);
    shootFrontRight.setDirection(DcMotor.Direction.REVERSE);
    shootBackLeft.setDirection(DcMotor.Direction.FORWARD);
    shootBackRight.setDirection(DcMotor.Direction.REVERSE);
  }

  ///////////////////////
  // Motor shenanigans //
  ///////////////////////

  // Left joystick dictates position, right dictates rotation
  private void handleMovement() {
    double verticalPower = -gamepad1.left_stick_y * currentSpeed;
    double horizontalPower = -gamepad1.left_stick_x * currentSpeed;
    double rotationalPower = gamepad1.right_stick_x * currentSpeed;

    // Lower value = slower front motors
    // Front wheels use HD Motors whereas back wheels use Core Motors
    final double FRONT_COMPENSATION = 1;
    driveFrontLeft.setPower((verticalPower + horizontalPower + rotationalPower) * FRONT_COMPENSATION);
    driveFrontRight.setPower((verticalPower - horizontalPower - rotationalPower) * FRONT_COMPENSATION);
    driveBackLeft.setPower(verticalPower - horizontalPower + rotationalPower);
    driveBackRight.setPower(verticalPower + horizontalPower - rotationalPower);
  }

  private void handleShooting() {
    final double SHOOT_POWER = 1;

    if (gamepad1.y) {
      powerFrontShootMotors(SHOOT_POWER);
    } else {
      powerFrontShootMotors(0);
    }

    if (gamepad1.b) {
      powerBackShootMotors(SHOOT_POWER);
    } else if (gamepad1.a) {
      powerBackShootMotors(-SHOOT_POWER);
    } else {
      powerBackShootMotors(0);
    }
  }

  private void powerFrontShootMotors(double power) {
    shootFrontLeft.setPower(power);
    shootFrontRight.setPower(power);
  }

  private void powerBackShootMotors(double power) {
    shootBackLeft.setPower(power);
    shootBackRight.setPower(power);
  }

  ///////////////////////
  // Speed Shenanigans //
  ///////////////////////

  private void handleSpeedControl() {
    final double SPEED_INCREMENT = 0.1;
    final double MIN_SPEED = 0.2;
    final double MAX_SPEED = 1;

    if (!isSpeedCooldownComplete()) {
      return;
    }

    if (gamepad1.dpad_up) {
      currentSpeed = Math.min(currentSpeed + SPEED_INCREMENT, MAX_SPEED);
      updateSpeedCooldown();
    } else if (gamepad1.dpad_down) {
      currentSpeed = Math.max(currentSpeed - SPEED_INCREMENT, MIN_SPEED);
      updateSpeedCooldown();
    }
  }

  private boolean isSpeedCooldownComplete() {
    final long SPEED_COOLDOWN_MILLIS = 300; 
    
    if (!gamepad1.dpad_up && !gamepad1.dpad_down) {
      return false;
    }
    return (System.currentTimeMillis() - lastSpeedChangeMillis) >= SPEED_COOLDOWN_MILLIS;
  }

  private void updateSpeedCooldown() {
    lastSpeedChangeMillis = System.currentTimeMillis();
  }

  // Logs data as visible in the controller
  private void updateTelemetry() {
    // List<AprilTagDetection> aprilTagDetections = aprilTagProcessor.getDetections();
    // for (AprilTagDetection aprilTagDetection : aprilTagDetections) {
    //  telemetry.addData("Tag ID", aprilTagDetection.id);
    // }

    telemetry.addData("Speed", currentSpeed);
    telemetry.addData("FL Power", driveFrontLeft.getPower());
    telemetry.addData("FR Power", driveFrontRight.getPower());
    telemetry.addData("BL Power", driveBackLeft.getPower());
    telemetry.addData("BR Power", driveBackRight.getPower());
    telemetry.update();
  }
}

/*
 * Motor configuration
 * 
 * driveFrontLeft = control hub 0
 * driveFrontRight = control hub 1
 * driveBackLeft = expansion hub 3
 * driveBackRight = expansion hub 2
 * 
 * ShootWheelFrontLeft = expansion hub 1
 * shootWheelFrontRight = expansion hub 0
 * shootWheelBackLeft = control hub 3
 * shootWheelBackRight = control hub 2
 */
