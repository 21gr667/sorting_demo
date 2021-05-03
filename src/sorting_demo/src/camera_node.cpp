#include <ros/ros.h>
#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
#include <zbar.h>
#include <iostream>

// Struct to store a set of frames from realsense
struct Frame {
    rs2::frameset frameset;
    rs2::frame colorFrame;
    rs2::frame depthFrame;
    uint height{};
    uint width{};
    cv::Mat matImage;
    uint32_t count = 0;
};

// Struct to store the scanned QR codes
struct Object : public cv::_InputArray {
    std::string data;
    std::vector<cv::Point> location;
    cv::Point center;
};

// Struct used to store information about each contour
struct features {
    int contourIndex;
    int area;
};

// Add names for QR codes here if you add more QR codes to scene
const std::vector<std::string> qrCustomNames = { "ID_1", "ID_2", "ID_3", "ID_4" };

// ROS service client
//ros::ServiceClient client;

// Find and decode barcodes and QR codes
void decode(cv::Mat& im, std::vector<Object>& decodedObjects) {

    // Create zbar scanner
    zbar::ImageScanner scanner;

    // Configure scanner
    scanner.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 1);

    // Convert image to grayscale
    cv::Mat imGray;
    cvtColor(im, imGray, cv::
        COLOR_BGR2GRAY);

    // Wrap image data in a zbar image
    zbar::Image image(im.cols, im.rows, "GREY", (uchar*)imGray.data, im.cols * im.rows);

    // Scan the image for barcodes and QRCodes
    int n = scanner.scan(image);

    // Print results
    for (zbar::Image::SymbolIterator symbol = image.symbol_begin(); symbol != image.symbol_end(); ++symbol) {
        Object obj;
        obj.data = symbol->get_data();

        // Obtain location
        for (int i = 0; i < symbol->get_location_size(); i++) {
            obj.location.emplace_back(symbol->get_location_x(i), symbol->get_location_y(i));
        }

        // calculate center coords
        int sumX = 0, sumY = 0;
        for (auto& i : obj.location) {
            sumX += i.x;
            sumY += i.y;
        }
        obj.center.x = sumX / obj.location.size();
        obj.center.y = sumY / obj.location.size();

        decodedObjects.push_back(obj);

    }
}

// Get a frame from realsense. Purpose of this function is to group the image collecting here.
void retrieveFrame(const rs2::pipeline& pipe, Frame* frame) {
    frame->frameset = pipe.wait_for_frames();                               // Get a frameset from realsense pipeline object. This object holds all data from the realsense camera
    frame->colorFrame = frame->frameset.get_color_frame();
    //frame->depthFrame = frame->frameset.get_depth_frame();                // We do not need depth frame for anything.. yet
    frame->width = frame->colorFrame.as<rs2::video_frame>().get_width();    // Width for OpenCV Mat object
    frame->height = frame->colorFrame.as<rs2::video_frame>().get_height();  // Height for OpenCV Mat object
    frame->matImage = cv::Mat(cv::Size(frame->width, frame->height), CV_8UC3, (void*)frame->colorFrame.get_data(),
        cv::Mat::AUTO_STEP);                          // Construct openCV mat object used in zBar and homogryaphy


// Increment frame number
    frame->count++;
}

void FindGraspingPoint(cv::Mat1b bin, cv::Mat hMatrix) { //Receives grayscale image   

    // Find contour
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(bin, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);


    //Creating vector<features> to store features
    std::vector<features> featVec;
    for (int i = 0; i < contours.size(); i++) {

        features f;
        f.contourIndex = i;

        //Store all features in featVec
        featVec.push_back(f);
        f.area = contourArea(contours[i]);
    }

    // Output image
    cv::Mat out;
    cv::cvtColor(bin, out, cv::COLOR_GRAY2BGR);
    
    //loop through contours
    for (int i = 0; i < contours.size(); i++) {
        if(featVec[i].area > 4000){
        // Draw on mask
        cv::Mat1b mask(bin.rows, bin.cols, uchar(0));
        cv::Mat1f dt;
        double max_val;
        cv::Point max_loc;
        cv::drawContours(mask, contours, featVec[i].contourIndex, cv::Scalar(255), cv::FILLED);

        // Distance Trasnsform
        cv::distanceTransform(mask, dt, cv::DIST_L2, 5, cv::DIST_LABEL_PIXEL);

        // Find max value
        cv::minMaxLoc(dt, nullptr, &max_val, nullptr, &max_loc);

        cv::circle(out, max_loc, max_val, cv::Scalar(0, 255, 0), 2);

        //lego_throw::camera camSrv;
        //camSrv.request.x = point.x;
        //camSrv.request.y = point.y;
        //camSrv.request.z = 0.05;
        //camSrv.request.data = max_loc;
        //camSrv.request.time = time_stamp;
      
        //if (client.call(camSrv)) printf("Response status: %i\n", camSrv.response.status);
        }
        std::cout<<featVec[i].area <<std::endl;

    }

    cv::imshow("Biggest Circles ", out);

}

cv::Mat calcHomography(const std::vector<Object>& objects, cv::Mat& colorImage) {
    
    // Find homography matrix needs 8 points.
    std::vector<cv::Point2f> surfaceQR(4); // Four corners of the real world plane
    std::vector<cv::Point2f> cameraQR(4);  // Four corners of the image plane

    // The QR codes that are scanned from zBar do not come ordered.
    // Thus we want to sort the corresponding points in SurfaceQR and cameraQR such that corresponding points is at the same index.
    int amountQRCornersFound = 0;

    // Assigning the cameraQR codes to the correct surfaceQR codes:
    for (int i = 0; i < objects.size(); ++i) {
        if (objects[i].data == "00") {
            surfaceQR[0] = cv::Point2f(0, 0);
            cameraQR[0] = objects[i].center;
            amountQRCornersFound++;
        }
        else if (objects[i].data == "01") {
            surfaceQR[1] = cv::Point2f(0, 262);
            cameraQR[1] = objects[i].center;
            amountQRCornersFound++;
        }
        else if (objects[i].data == "02") {
            surfaceQR[2] = cv::Point2f(462, 0);
            cameraQR[2] = objects[i].center;
            amountQRCornersFound++;
        }
        else if (objects[i].data == "03") {
            surfaceQR[3] = cv::Point2f(462, 262);
            cameraQR[3] = objects[i].center;
            amountQRCornersFound++;
        }
    }

    // Visualizing the found points:
    for (int i = 0; i < amountQRCornersFound; i++) {
        if (i == 0) cv::circle(colorImage, cameraQR[i], 5, cv::Scalar(255, 0, 0), -1); // BLUE
        if (i == 1) cv::circle(colorImage, cameraQR[i], 5, cv::Scalar(0, 255, 0), -1); // GREEN
        if (i == 2) cv::circle(colorImage, cameraQR[i], 5, cv::Scalar(0, 0, 255), -1); // RED
        if (i == 3) cv::circle(colorImage, cameraQR[i], 5, cv::Scalar(255, 255, 0), -1); // YELLOW
    }

    // Returning empty matrix if all the QR codes were not found:
    if (amountQRCornersFound != 4){
        std::cout << "amountQRCornersFound = "<< amountQRCornersFound << ": could not calculate the homography." << std::endl;
        cv::Mat empty;
        return empty;
    }

    // Calculate Homography matrix from 4 sets of corresponding points
    cv::Mat hMatrix = findHomography(cameraQR, surfaceQR);
    return hMatrix;
}

int main(int argc, char **argv) {

    std::cout << "Hello from camera_node!" << std::endl;

    // Starting Realsense2:
    rs2::pipeline pipe;
    pipe.start(); 
    // Declare depth colorizer for pretty visualization of depth data
    rs2::colorizer color_map;

    // Creating a node called camera_node:
    ros::init(argc, argv, "camera_node");
    ros::NodeHandle node_handle;

    // Creating the client
    //client = node_handle.serviceClient<lego_throw::camera>("camera");
    //client.waitForExistence();

    printf("Start filming the scene\n");

    while (ros::ok())
    {

        // Retrieving newest frame from the camera and converting the frame fra BGR to RGB:
        Frame frame;
        retrieveFrame(pipe, &frame);   
        cvtColor(frame.matImage, frame.matImage, cv::COLOR_BGR2RGB);    

        // Rotating image 180 degrees:
        cv::rotate(frame.matImage, frame.matImage, cv::ROTATE_180);

        // Finding QR codes in the frame:
        std::vector<Object> decodedObjects;
        decode(frame.matImage, decodedObjects);  

        // Trying to calculate the homography for the found QR codes:
        cv::Mat hMatrix = calcHomography(decodedObjects, frame.matImage);
        cv::imshow("RGB", frame.matImage);

        // Only if a hMatrix is returned:
        if (hMatrix.empty() != true) {
        
            cv::Mat homographyImage;
            cv::warpPerspective(frame.matImage, homographyImage, hMatrix, cv::Size(462, 400));
            cv::imshow("Homography image", homographyImage);
        
            // Images for thresholding:
            cv::Mat detectRed = homographyImage.clone();
            cv::Mat detectBlue = homographyImage.clone();
            cv::Mat detectYellow = homographyImage.clone();

            //Thresholding colors:
            cv::inRange(detectRed, cv::Scalar(0, 0, 120), cv::Scalar(65, 65, 175), detectRed); //R�d
            cv::inRange(detectBlue, cv::Scalar(100, 55, 0), cv::Scalar(170, 80, 20), detectBlue); //Bl�
            cv::inRange(detectYellow, cv::Scalar(0, 140, 140), cv::Scalar(70, 200, 200), detectYellow); //Gr�n
            
            // Displaying thresholds:
            cv::imshow("Red", detectRed);
            cv::imshow("Blue", detectBlue);
            cv::imshow("Yellow", detectYellow);

            // Morphology:
            cv::Mat elem = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(61, 61));
            cv::morphologyEx(detectRed, detectRed, cv::MORPH_OPEN, elem);
            cv::morphologyEx(detectRed, detectRed, cv::MORPH_CLOSE, elem);
            cv::morphologyEx(detectBlue, detectBlue, cv::MORPH_OPEN, elem);
            cv::morphologyEx(detectBlue, detectBlue, cv::MORPH_CLOSE, elem);
            cv::morphologyEx(detectYellow, detectYellow, cv::MORPH_OPEN, elem);
            cv::morphologyEx(detectYellow, detectYellow, cv::MORPH_CLOSE, elem);

        }
        /*
        
        
        //decode(frame.matImage, decodedObjects);  
        cv::Mat hMatrix;
        if (decodedObjects.size() > 3) {                                         
           hMatrix = doHomography(decodedObjects, &frame.matImage);

           if (!hMatrix.empty()) {
               FindGraspingPoint(detectGreen, hMatrix);
           }
        }  
        */
        if (cv::waitKey(25) == 27) break;  // If ESC is pushed then break loop
    }

    return 0;
}







