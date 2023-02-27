#include "darknet.hpp"

#include "opencv2/highgui/highgui.hpp"
#include <string>
#include <chrono>

#define DETECTION_THRESHOLD         0.24 // Minimum probability threshold for detection
#define DETECTION_HIER_THRESHOLD    0.5  // Minimum hierarchy threshold for detection
#define NMS_THRESHOLD               0.4  // Non-maximum suppression threshold for detection

int main(int argc, char *argv[])
{
    cv::VideoCapture cap; // OpenCV object to capture video frames
    cv::Mat cvimage;      // OpenCV object to hold video frames
    Darknet::Image dnimage; // Darknet object to hold video frames
    Darknet::ConvertCvBgr8 converter; // Object to convert OpenCV images to Darknet images
    Darknet::Detector detector; // Object to run Darknet detections
    std::vector<Darknet::Detection> detections; // Vector to hold detected objects

    if (argc < 4) { // Check if required arguments are provided
        std::cerr << "Usage: " << argv[0] << " <input_names_file> <input_cfg_file> <input_weights_file> [<videofile>]" << std::endl;
        return -1;
    }

    std::string input_names_file(argv[1]); // File path to Darknet object names
    std::string input_cfg_file(argv[2]);   // File path to Darknet configuration file
    std::string input_weights_file(argv[3]); // File path to Darknet weights file

    if (argc == 5) { // Check if video file argument is provided
        std::string videofile(argv[4]); // File path to video file
        if (!cap.open(videofile)) { // Open the video file
            std::cerr << "Could not open video file" << std::endl;
            return -1;
        }
    } else if (!cap.open(0)) { // Open the default camera if no video file is provided
        std::cerr << "Could not open video input stream" << std::endl;
        return -1;
    }

    int image_width = cap.get(cv::CAP_PROP_FRAME_WIDTH); // Get the width of the video frames
    int image_height = cap.get(cv::CAP_PROP_FRAME_HEIGHT); // Get the height of the video frames

    // Setup the Darknet detector with the provided configuration and weights files
    if (!detector.setup(input_names_file,
                        input_cfg_file,
                        input_weights_file,
                        NMS_THRESHOLD,
                        DETECTION_THRESHOLD,
                        DETECTION_HIER_THRESHOLD,
                        image_width,
                        image_height)) {
        std::cerr << "Setup failed" << std::endl;
        return -1;
    }

    // Setup the converter object with the dimensions of the video frames and Darknet object dimensions
    converter.setup(image_width, image_height, detector.get_width(), detector.get_height());
    auto prevTime = std::chrono::system_clock::now(); // Start timer

    while(1) { // Loop to capture and detect objects in video frames

        if (!cap.read(cvimage)) { // Read a frame from the video capture
            std::cerr << "Video capture read failed/EoF" << std::endl;
            return -1;
        }

        // Convert and resize the OpenCV image to a Darknet image
        if (!converter.convert(cvimage, dnimage)) {
            std::cerr << "Failed to convert opencv image to darknet image" << std::endl;
            return -1;
        }

        // Run the Darknet
        while(1) {

        // read the next frame from the video stream
        if (!cap.read(cvimage)) {
            std::cerr << "Video capture read failed/EoF" << std::endl;
            return -1;
        }

        // convert the OpenCV image to a Darknet image and resize it
        if (!converter.convert(cvimage, dnimage)) {
            std::cerr << "Failed to convert opencv image to darknet image" << std::endl;
            return -1;
        }

        // run the object detector on the Darknet image
        if (!detector.detect(dnimage)) {
            std::cerr << "Failed to run detector" << std::endl;
            return -1;
        }

        // get the detections from the detector
        detector.get_detections(detections);

        // overlay the bounding boxes on the OpenCV image
        Darknet::image_overlay(detections, cvimage);

        // calculate the FPS and print it to the console
        auto now = std::chrono::system_clock::now();
        std::chrono::duration<double> period = (now - prevTime);
        prevTime = now;
        std::cout << "FPS: " << 1 / period.count() << std::endl;

        // display the image with the bounding boxes
        cv::imshow("Overlay", cvimage);
        cv::waitKey(1);

}