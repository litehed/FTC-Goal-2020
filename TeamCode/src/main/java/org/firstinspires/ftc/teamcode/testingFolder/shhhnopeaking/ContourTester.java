package org.firstinspires.ftc.teamcode.testingFolder.shhhnopeaking;

import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class ContourTester extends OpenCvPipeline {

    Scalar low = new Scalar(0.0, 141.0, 0.0);
    Scalar high = new Scalar(255.0, 230.0, 95.0);

    ColorContour colorContour = new ColorContour(low, high);

    @Override
    public Mat processFrame(Mat input) {
        Mat frame = new Mat();

        if(!frame.empty()){
            Mat mask = new Mat();
            Mat output = new Mat();

            Imgproc.cvtColor(input, output, Imgproc.COLOR_RGB2YCrCb);
            mask = new Mat(output.rows(), output.cols(), CvType.CV_8UC1);

            colorContour.findAndDrawContours(output, frame);
        }
        return frame;
    }
}
