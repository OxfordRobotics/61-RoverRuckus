package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class HardwareMapJack
{
    /* Public OpMode members. */
    public DcMotor  motorLeft = null;
    public DcMotor  motorRight = null;
    /*
    public DcMotor  motorLift = null;
    public DcMotor  extendo = null;
    public DcMotor  motorElbow = null;
    public CRServo  ColServo = null;
    public Servo    Bucket = null;

    /* local OpMode members. */
    com.qualcomm.robotcore.hardware.HardwareMap hwMap = null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public HardwareMapJack(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(com.qualcomm.robotcore.hardware.HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        motorLeft = hwMap.get(DcMotor.class, "motorLeft");
        motorRight = hwMap.get(DcMotor.class, "motorRight");
        /*

        motorLift = hwMap.get(DcMotor.class, "Motor3");

        motorElbow = hwMap.get(DcMotor.class, "Motor4");
        extendo = hwMap.get(DcMotor.class, "extendmotor");
        ColServo = hwMap.get(CRServo.class, "colservo");
        Bucket = hwMap.get(Servo.class, "bucket");
*/
        motorLeft.setDirection(DcMotor.Direction.REVERSE);
        motorRight.setDirection(DcMotor.Direction.FORWARD);

        /*
        motorLift.setDirection(DcMotor.Direction.FORWARD);
        motorElbow.setDirection(DcMotor.Direction.FORWARD);
        extendo.setDirection(DcMotor.Direction.FORWARD);
        ColServo.setDirection(CRServo.Direction.FORWARD);
        Bucket.setDirection(Servo.Direction.FORWARD);
        */
        // Set all motors to zero power
        motorLeft.setPower(0);
        motorRight.setPower(0);
        /*
        motorLift.setPower(0);
        motorElbow.setPower(0);
        extendo.setPower(0);
        */
        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        motorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        /*motorLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        /*
        motorElbow.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        extendo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // Define and initialize ALL installed servos.
*/
   }
 }


