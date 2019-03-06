/*
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

@TeleOp(name="4 motors", group="Iterative Opmode")
public class Basic_Program extends OpMode {

    HardwareMap hwMap = new HardwareMap();
    private ElapsedTime runtime = new ElapsedTime();

    /*
     * Code to run ONCE when the driver hits INIT
     */ /*
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        hwMap.init(hardwareMap);
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */ /*
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY

    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP


    public void loop() {
         hwMap.Motor1.setPower(-gamepad1.left_stick_y);
         hwMap.Motor4.setPower(-gamepad1.left_stick_y);
         hwMap.Motor3.setPower(gamepad1.right_stick_y);
         hwMap.Motor2.setPower(gamepad1.right_stick_y);

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
    }

    /*
     * Code to run ONCE after the driver hits STOP

    @Override
    public void stop() {
    }

}
*/