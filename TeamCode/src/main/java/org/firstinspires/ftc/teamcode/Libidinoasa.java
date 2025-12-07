package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;


@TeleOp
@Disabled
public class Libidinoasa extends LinearOpMode {
    private DcMotorEx shooter;
    PidControllerAdevarat pid = new PidControllerAdevarat(0, 0, 0);
    boolean ok = false, a = false, abis = false;
    double pidResultS;

    @Override
    public void runOpMode() throws InterruptedException {
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooter.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        pid.enable();
        waitForStart();
        while (opModeIsActive()) {
            if (isStopRequested()) return;
            pid.setPID(0.8, 0.0, 0.0);
            double velo = shooter.getVelocity();
            telemetry.addData("shoot", velo);
            telemetry.update();

            if (gamepad1.a) {
//                shooter.setVelocity(10000);
                pidResultS = pid.performPID(2000);
                shooter.setVelocity(pidResultS);
                ok = true;
            } else {
                if (ok) {
                    ok = false;
                    pid.setSetpoint(2000);
                }
            }
            if (gamepad1.b) {
                shooter.setVelocity(0);
            }

//                    if (s.touch_slider.isPressed()) {
//                        pid.setSetpoint(0);
//                        s.sliderD.setPower(0);
//                        s.sliderS.setPower(0);
//                    } else {
//                        pidResultS = pidSlider.performPID(s.sliderD.getCurrentPosition());
//                        //  s.slider1.setPower(pidResultS);
//                        //  s.slider2.setPower(pidResultS);
//                        s.sliderD.setPower(pidResultS);
//                        s.sliderS.setPower(pidResultS);
//                    }
        }
    }
}