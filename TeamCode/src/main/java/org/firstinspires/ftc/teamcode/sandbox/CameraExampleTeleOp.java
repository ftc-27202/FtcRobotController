/*
 * Copyright (c) 2019 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.sandbox;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

// This version of the internal camera example uses EasyOpenCV's interface to the original
// Android camera API

@TeleOp(name = "Camera Example TeleOp", group = "Robot")

public class CameraExampleTeleOp extends LinearOpMode
{
	OpenCvCamera phoneCam;

	@Override
	public void runOpMode()
	{
		int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
			"cameraMonitorViewId",
			"id",
			hardwareMap.appContext.getPackageName());

		phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(
			OpenCvInternalCamera.CameraDirection.BACK,
			cameraMonitorViewId);

		// Specify the image processing pipeline we wish to invoke upon receipt of a frame from the camera.
		// Note that switching pipelines on-the-fly (while a streaming session is in flight) *IS* supported.
		phoneCam.setPipeline(new SkystoneDeterminationPipeline());

		phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
		{
			@Override
			public void onOpened()
			{
				phoneCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
			}

			@Override
			public void onError(int errorCode)
			{
				// This will be called if the camera could not be opened
			}
		});

		telemetry.addLine("Waiting for start");
		telemetry.update();

		// Wait for the user to press start on the Driver Station
		waitForStart();

		while (opModeIsActive())
		{
			// Send some stats to the telemetry
			telemetry.addData("Frame Count", phoneCam.getFrameCount());
			telemetry.addData("FPS", String.format("%.2f", phoneCam.getFps()));
			telemetry.addData("Total frame time ms", phoneCam.getTotalFrameTimeMs());
			telemetry.addData("Pipeline time ms", phoneCam.getPipelineTimeMs());
			telemetry.addData("Overhead time ms", phoneCam.getOverheadTimeMs());
			telemetry.addData("Theoretical max FPS", phoneCam.getCurrentPipelineMaxFps());
			telemetry.update();

			if (gamepad1.a)
			{
				// IMPORTANT NOTE: calling stopStreaming() will indeed stop the stream of images
				// from the camera (and, by extension, stop calling your vision pipeline). HOWEVER,
				// if the reason you wish to stop the stream early is to switch use of the camera
				// over to, say, Vuforia or TFOD, you will also need to call closeCameraDevice()
				// (commented out below), because according to the Android Camera API documentation:
				//	     "Your application should only have one Camera object active at a time for
				//	      a particular hardware camera."
				//
				// NB: calling closeCameraDevice() will internally call stopStreaming() if applicable,
				// but it doesn't hurt to call it anyway, if for no other reason than clarity.
				//
				// NB2: if you are stopping the camera stream to simply save some processing power
				// (or battery power) for a short while when you do not need your vision pipeline,
				// it is recommended to NOT call closeCameraDevice() as you will then need to re-open
				// it the next time you wish to activate your vision pipeline, which can take a bit of
				// time. Of course, this comment is irrelevant in light of the use case described in
				// the above "important note".
				phoneCam.stopStreaming();
				//phoneCam.closeCameraDevice();
			}

			sleep(100);
		}
	}

	static class SkystoneDeterminationPipeline extends OpenCvPipeline
	{
		// An enum to define the skystone position
		enum SkystonePosition
		{LEFT, CENTER, RIGHT}

		// Some color constants
		final Scalar BLUE = new Scalar(0, 0, 255);
		final Scalar GREEN = new Scalar(0, 255, 0);

		// The core values which define the location and size of the sample regions
		final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(50, 130);
		final Point REGION2_TOPLEFT_ANCHOR_POINT = new Point(110, 130);
		final Point REGION3_TOPLEFT_ANCHOR_POINT = new Point(170, 130);
		static final int REGION_WIDTH = 20;
		static final int REGION_HEIGHT = 20;

		// Points which actually define the sample region rectangles, derived from above values
		Point region1_pointA = new Point(
			REGION1_TOPLEFT_ANCHOR_POINT.x,
			REGION1_TOPLEFT_ANCHOR_POINT.y);
		Point region1_pointB = new Point(
			REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
			REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
		Point region2_pointA = new Point(
			REGION2_TOPLEFT_ANCHOR_POINT.x,
			REGION2_TOPLEFT_ANCHOR_POINT.y);
		Point region2_pointB = new Point(
			REGION2_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
			REGION2_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
		Point region3_pointA = new Point(
			REGION3_TOPLEFT_ANCHOR_POINT.x,
			REGION3_TOPLEFT_ANCHOR_POINT.y);
		Point region3_pointB = new Point(
			REGION3_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
			REGION3_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

		// Working variables
		Mat region1_Cb, region2_Cb, region3_Cb;
		Mat YCrCb = new Mat();
		Mat Cb = new Mat();
		int avg1, avg2, avg3;

		// Volatile since accessed by OpMode thread w/o synchronization
		private volatile SkystonePosition position = SkystonePosition.LEFT;

		// This function takes the RGB frame, converts to YCrCb, and extracts the Cb channel to the
		// 'Cb' variable
		void inputToCb(Mat input)
		{
			Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
			Core.extractChannel(YCrCb, Cb, 2);
		}

		@Override
		public void init(Mat firstFrame)
		{
			// We need to call this in order to make sure the 'Cb' object is initialized, so that the
			// submats we make will still be linked to it on subsequent frames. (If the object were to
			// only be initialized in processFrame, then the submats would become delinked because the
			// backing buffer would be re-allocated the first time a real frame was crunched)
			inputToCb(firstFrame);

			// Submats are a persistent reference to a region of the parent buffer. Any changes to the
			// child affect the parent, and the reverse also holds true.
			region1_Cb = Cb.submat(new Rect(region1_pointA, region1_pointB));
			region2_Cb = Cb.submat(new Rect(region2_pointA, region2_pointB));
			region3_Cb = Cb.submat(new Rect(region3_pointA, region3_pointB));
		}

		@Override
		public Mat processFrame(Mat input)
		{
			// We first convert to YCrCb color space, from RGB color space.
			// Why do we do this? Well, in the RGB color space, chroma and
			// luma are intertwined. In YCrCb, chroma and luma are separated.
			// YCrCb is a 3-channel color space, just like RGB. YCrCb's 3 channels
			// are Y, the luma channel (which essentially just a B&W image), the
			// Cr channel, which records the difference from red, and the Cb channel,
			// which records the difference from blue. Because chroma and luma are
			// not related in YCrCb, vision code written to look for certain values
			// in the Cr/Cb channels will not be severely affected by differing
			// light intensity, since that difference would most likely just be
			// reflected in the Y channel.
			//
			// After we've converted to YCrCb, we extract just the 2nd channel, the
			// Cb channel. We do this because stones are bright yellow and contrast
			// STRONGLY on the Cb channel against everything else, including SkyStones
			// (because SkyStones have a black label).
			//
			// We then take the average pixel value of 3 different regions on that Cb
			// channel, one positioned over each stone. The brightest of the 3 regions
			// is where we assume the SkyStone to be, since the normal stones show up
			// extremely darkly.
			//
			// We also draw rectangles on the screen showing where the sample regions
			// are, as well as drawing a solid rectangle over top the sample region
			// we believe is on top of the SkyStone.
			//
			// In order for this whole process to work correctly, each sample region
			// should be positioned in the center of each of the first 3 stones, and
			// be small enough such that only the stone is sampled, and not any of the
			// surroundings.

			// Get the Cb channel of the input frame after conversion to YCrCb
			inputToCb(input);

			// Compute the average pixel value of each submat region. We're taking the average of a
			// single channel buffer, so the value we need is at index 0. We could have also taken the
			// average pixel value of the 3-channel image, and referenced the value at index 2 here.
			avg1 = (int) Core.mean(region1_Cb).val[0];
			avg2 = (int) Core.mean(region2_Cb).val[0];
			avg3 = (int) Core.mean(region3_Cb).val[0];

			// Draw a rectangle showing sample region 1 on the screen. Simply a visual aid. Serves no
			// functional purpose.
			Imgproc.rectangle(
				input, // Buffer to draw on
				region1_pointA, // First point which defines the rectangle
				region1_pointB, // Second point which defines the rectangle
				BLUE, // The color the rectangle is drawn in
				2); // Thickness of the rectangle lines

			// Draw a rectangle showing sample region 2 on the screen. Simply a visual aid. Serves no
			// functional purpose.
			Imgproc.rectangle(
				input, // Buffer to draw on
				region2_pointA, // First point which defines the rectangle
				region2_pointB, // Second point which defines the rectangle
				BLUE, // The color the rectangle is drawn in
				2); // Thickness of the rectangle lines

			// Draw a rectangle showing sample region 3 on the screen. Simply a visual aid. Serves no
			// functional purpose.
			Imgproc.rectangle(
				input, // Buffer to draw on
				region3_pointA, // First point which defines the rectangle
				region3_pointB, // Second point which defines the rectangle
				BLUE, // The color the rectangle is drawn in
				2); // Thickness of the rectangle lines

			// Find the max of the 3 averages
			int maxOneTwo = Math.max(avg1, avg2);
			int max = Math.max(maxOneTwo, avg3);

			// Now that we found the max, we actually need to go and figure out which sample region
			// that value was from
			if (max == avg1)
			{
				position = SkystonePosition.LEFT; // Record our analysis

				// Draw a solid rectangle on top of the chosen region.
				// Simply a visual aid. Serves no functional purpose.
				Imgproc.rectangle(
					input, // Buffer to draw on
					region1_pointA, // First point which defines the rectangle
					region1_pointB, // Second point which defines the rectangle
					GREEN, // The color the rectangle is drawn in
					-1); // Negative thickness means solid fill
			} else if (max == avg2)
			{
				position = SkystonePosition.CENTER; // Record our analysis

				// Draw a solid rectangle on top of the chosen region. Simply a visual aid. Serves no
				// functional purpose.
				Imgproc.rectangle(
					input, // Buffer to draw on
					region2_pointA, // First point which defines the rectangle
					region2_pointB, // Second point which defines the rectangle
					GREEN, // The color the rectangle is drawn in
					-1); // Negative thickness means solid fill
			} else // max == avg3
			{
				position = SkystonePosition.RIGHT; // Record our analysis

				// Draw a solid rectangle on top of the chosen region. Simply a visual aid. Serves no
				// functional purpose.
				Imgproc.rectangle(
					input, // Buffer to draw on
					region3_pointA, // First point which defines the rectangle
					region3_pointB, // Second point which defines the rectangle
					GREEN, // The color the rectangle is drawn in
					-1); // Negative thickness means solid fill
			}

			// Render the 'input' buffer to the viewport. But note this is not simply rendering the raw
			// camera feed, because we called functions to add some annotations to this buffer earlier up.
			return input;
		}

		// Call this from the OpMode thread to obtain the latest analysis
		public SkystonePosition getAnalysis()
		{
			return position;
		}
	}
}
