#include <ros/ros.h>
#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
#include <zbar.h>
#include <iostream>
#include <geometry_msgs/Point.h>
#include <sorting_demo/object.h>

#define CONVEYOR_SPEED 25.15;

int wasteCounter = 0;

ros::Publisher wastePublisher;

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

struct waste {
    int id;
    ros::Time timeStamp;
    ros::Time creationTime;
    cv::Mat mask;
    std::string type;
};

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

void morphology(cv::Mat &image, cv::Mat &elem) {
    cv::morphologyEx(image, image, cv::MORPH_OPEN, elem);
    cv::morphologyEx(image, image, cv::MORPH_CLOSE, elem);
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

cv::Point findGraspingPoint(cv::Mat1b bin) { //Receives grayscale image   

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
    
    cv::Point max_loc;

    //loop through contours
    for (int i = 0; i < contours.size(); i++) {
        
        //if(featVec[i].area > 50){
        // Draw on mask
        cv::Mat1b mask(bin.rows, bin.cols, uchar(0));
        cv::Mat1f dt;
        double max_val;
        
        cv::drawContours(mask, contours, featVec[i].contourIndex, cv::Scalar(255), cv::FILLED);

        // Distance Trasnsform
        cv::distanceTransform(mask, dt, cv::DIST_L2, 5, cv::DIST_LABEL_PIXEL);

        // Find max value
        cv::minMaxLoc(dt, nullptr, &max_val, nullptr, &max_loc);

        cv::circle(out, max_loc, max_val, cv::Scalar(0, 255, 0), 5);
        
        
        //}


        //std::cout<<featVec[i].area <<std::endl;

    }

    cv::imshow("Biggest Circles ", out);

    return max_loc;

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
        //std::cout << "amountQRCornersFound = "<< amountQRCornersFound << ": could not calculate the homography." << std::endl;
        cv::Mat empty;
        return empty;
    }

    // Calculate Homography matrix from 4 sets of corresponding points
    cv::Mat hMatrix = findHomography(cameraQR, surfaceQR);
    return hMatrix;
}

void findWasteObjects(cv::Mat image, ros::Time timeStamp, std::string type,  std::vector<waste> &wasteObjects) {

    // Finding contours in an image:
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(image, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    // First we check if there were any contours in the image:
    if (contours.empty()) {
        //std::cout << "No contours." << std::endl;
        return;
    }

    // We check if there are masks of the relevant type in the vector:
    // We find the relevant masks of the same type:
    std::vector<int> wasteIndex;
    bool typeFound = false;
    for (int i = 0; i < wasteObjects.size(); i++) {
        if (wasteObjects[i].type == type) {
            wasteIndex.push_back(i);
            typeFound = true;
        }
    }

    // If there is already waste in the vector, we need to ensure that we do not have multiple of the same objects:
    // We need to update the masks ensure no multiple of the same object:
    bool anyOverlap = false;

    if (wasteObjects.size() != 0 and typeFound == true) {
            
        // Now we check if any of the newly found objects to see, if the overlap with already found objects:
        for (int i = 0; i < contours.size(); i++) {

            // Mask:
            cv::Mat mask = cv::Mat::zeros(cv::Size(image.size()), CV_8U);

            // Filling the contour:
            cv::drawContours(mask, contours, i, 255, -1);

            // If any of the masks overlap:
            anyOverlap = false;


            // We check if the mask overlaps one of our objects:
            for (int i = 0; i < wasteIndex.size(); i++) {
                
                // First we enlarge our mask:
                cv::Size maskSize = wasteObjects[wasteIndex[i]].mask.size();
                int yOffset = maskSize.height - 150;

                // Now we check if our mask and the wasteObjectsUpdated mask overlaps:
                bool overlap = false;
                    
                for (int x = 0; x < mask.cols; x++) {
				    for (int y = 0; y < mask.rows; y++) { 
					    if (wasteObjects[wasteIndex[i]].mask.at<uchar>(cv::Point(x, y + yOffset)) == 255 && mask.at<uchar>(cv::Point(x, y)) == 255) {
                            overlap = true;
						    anyOverlap = true;
                            break;
					    }
				    }
			    }

                // If we had overlap, we need to update the mask and time stamp:
                if (overlap == true) {

                    // We update the mask:
                    for (int x = 0; x < mask.cols; x++) {
				        for (int y = 0; y < mask.rows; y++) {
					        if (wasteObjects[wasteIndex[i]].mask.at<uchar>(cv::Point(x, y + yOffset)) == 0 && mask.at<uchar>(cv::Point(x, y)) == 255) {
                                wasteObjects[wasteIndex[i]].mask.at<unsigned char>(cv::Point(x, y + yOffset)) = 255; 
                            } 
				        }
			        }

                    // We update our object:
                    wasteObjects[wasteIndex[i]].timeStamp = timeStamp;
                }
            }
            
            if (anyOverlap == false) {

                // Creating the struct with the object:
                waste temp_object;
                temp_object.id = wasteCounter++;
                temp_object.timeStamp = timeStamp;
                temp_object.creationTime = timeStamp;
                temp_object.mask = mask;
                temp_object.type = type;

                // Appending the object:
                wasteObjects.push_back(temp_object);
            }
        }
    }
    
    // If there are no objects in waste object, we simply add the object to the vector:
    else if (wasteObjects.size() == 0 || typeFound == false || anyOverlap == false) {

        //std::cout << "NEW MASK" << std::endl;

        for (int i = 0; i < contours.size(); i++) {

            // Mask:
            cv::Mat mask = cv::Mat::zeros(cv::Size(image.size()), CV_8U);

            // Filling the contour:
            cv::drawContours(mask, contours, i, 255, -1);

            // Creating the struct with the object:
            waste temp_object;
            temp_object.id = wasteCounter++;
            temp_object.timeStamp = timeStamp;
            temp_object.creationTime = timeStamp;
            temp_object.mask = mask;
            temp_object.type = type;

            // Appending the object:
            wasteObjects.push_back(temp_object);
        }
    }
}

cv::Mat plotWasteObjects(std::vector<waste> wasteObjects) {

    cv::Mat image = cv::Mat::zeros(cv::Size(265, 750), CV_8UC3);

    // Loop through waste objects:
    for (int i = 0; i < wasteObjects.size(); i++) {
        std::cout << i << std::endl;

        cv::Size maskSize = wasteObjects[i].mask.size();

        int yOffset = 750 - maskSize.height;

        // Loop through the image:
        for (int x = 0; x < wasteObjects[i].mask.cols; x++) {
		    for (int y = 0; y < wasteObjects[i].mask.rows; y++) {

                if (wasteObjects[i].mask.at<uchar>(cv::Point(x, y)) == 255) {

                    if (wasteObjects[i].type == "red") {
                        image.at<cv::Vec3b>(cv::Point(x, yOffset + y))[2] = 255;
                    }

                    else if (wasteObjects[i].type == "blue") {
                        image.at<cv::Vec3b>(cv::Point(x, yOffset + y))[0] = 255;
                    }

                    else if (wasteObjects[i].type == "yellow") {
                        image.at<cv::Vec3b>(cv::Point(x, yOffset + y))[1] = 255;
                        image.at<cv::Vec3b>(cv::Point(x, yOffset + y))[2] = 255;
                    }
                }
            }
        }
    }

    return image;
}

void removeOverlapMasks(std::vector<waste> &wasteObjects) {

    if (wasteObjects.size() == 0) {
        return;
    }

    for (int i = 0; i < wasteObjects.size()-1; i++) {
        for (int j = i+1; j < wasteObjects.size(); j++) {

            if (i == j) break;

            bool overlap = false;

            cv::Size mask1Size = wasteObjects[i].mask.size();
            cv::Size mask2Size = wasteObjects[j].mask.size();

            if (mask1Size.width >= mask2Size.width) {

                int yOffset =  mask1Size.width - mask2Size.width;

                for (int x = 0; x < wasteObjects[j].mask.cols; x++) {
				    for (int y = 0; y < wasteObjects[j].mask.rows; y++) {
                            
					    if (wasteObjects[i].mask.at<uchar>(cv::Point(x, y + yOffset)) == 255 && wasteObjects[j].mask.at<uchar>(cv::Point(x, y)) == 255) {
                            wasteObjects.erase(wasteObjects.begin() + j);
                            j = j - 1;
                            overlap = true;
                            break; 
					    }
				    }
                    if (overlap == true) break;
			    }
            }

            else {

                int yOffset =  mask2Size.width - mask1Size.width;

                for (int x = 0; x < wasteObjects[i].mask.cols; x++) {
				    for (int y = 0; y < wasteObjects[i].mask.rows; y++) {
                            
					    if (wasteObjects[i].mask.at<uchar>(cv::Point(x, y)) == 255 && wasteObjects[j].mask.at<uchar>(cv::Point(x, y + yOffset)) == 255) {
                            wasteObjects.erase(wasteObjects.begin() + j);
                            j = j - 1;
                            overlap = true;
                            break; 
					    }
				    }
                    if (overlap == true) break;
			    }
            }  
        }
    }
}

void removePassedObjects(std::vector<waste> &wasteObjects) {

    if (wasteObjects.size() == 0) return;

    for (int i = 0; i < wasteObjects.size(); i++) {


        int id = wasteObjects[i].id;
        double elapsed_time = ros::Time::now().toSec() - wasteObjects[i].creationTime.toSec();

        if (elapsed_time > 10) {

            // Finding picking point:
            cv::Point pickPoint = findGraspingPoint(cv::Mat1b(wasteObjects[i].mask));
            std::cout << "Picking point: " << pickPoint << std::endl;

            // Time passed:
            double elapsedTime = ros::Time::now().toSec() - wasteObjects[i].creationTime.toSec();

            int x = pickPoint.x + 85 + (126/2);
            int y = (-1 * pickPoint.y) + ((1000.0 / 25.15) * elapsedTime);

            std::cout << "Calculated X: " << x << ", Y: " << y << ", time: " << elapsedTime <<std::endl;

            // Sending the object:
            sorting_demo::object msg;
            geometry_msgs::Point point;
            point.x = x;
            point.y = y;
            point.z = 0;
            msg.point = point;
            msg.stamp = ros::Time::now();
            msg.type = wasteObjects[i].type;
            wastePublisher.publish(msg);

            // Remove the mask:
            wasteObjects.erase(wasteObjects.begin() + i);

            // Decrement i if the are NOT at the end if the array:
            if (i != wasteObjects.size()) i = i - 1;

            std::cout << "Removed: " << id << std::endl;
        }
    }
}

void updateMasks(std::vector<waste> &wasteObjects){

    // First we update the location of the masks for the relevant waste types:
    for (int i = 0; i < wasteObjects.size(); i++) {

        // We grap a copy of the object:
        waste tempWaste; 
        tempWaste.timeStamp = wasteObjects[i].timeStamp;
        tempWaste.mask = wasteObjects[i].mask.clone();

        // We calcute how far the object has moved:
        double elapsed_time = ros::Time::now().toSec() - tempWaste.timeStamp.toSec();
        double distance  = round((1000.0 / 25.15) * elapsed_time);

        // We create a temporay mask:
        cv::Size maskSize = tempWaste.mask.size();
        cv::Mat tempMask = cv::Mat::zeros(cv::Size(maskSize.width, maskSize.height + distance), CV_8U);

        // Setting mask tempWaste with offset:
        cv::Mat roi = tempMask(cv::Rect(0, 0, maskSize.width, maskSize.height));
        tempWaste.mask.copyTo(roi);
                
        // We set the mask to the new updated mask:
        tempWaste.mask = tempMask.clone();

        // We push back the updated object:
        wasteObjects[i].mask = tempWaste.mask.clone();
        wasteObjects[i].timeStamp = ros::Time::now();
    }
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

    wastePublisher = node_handle<sorting_demo::object>("objects", 1);

    // Queue with waste objects:
    std::vector<waste> wasteObjects;

    // Structuring element for morphology:
    cv::Mat elem = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(15, 15));

    ros::Rate loop_rate(3);

    printf("Start filming the scene\n");

    while (ros::ok()){

        // Retrieving newest frame from the camera and converting the frame fra BGR to RGB:
        Frame frame;
        retrieveFrame(pipe, &frame);   
        cvtColor(frame.matImage, frame.matImage, cv::COLOR_BGR2RGB);  

        if (frame.matImage.empty() == true) {
            std::cout << "Camera frame empty..." << std::endl;
            continue;
        }

        // We get the time stamp for the image:
        ros::Time timeStamp = ros::Time::now();

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
            //cv::imshow("Homography image", homographyImage);
        
            // Cropping the homography image:
            cv::Mat objectImage = homographyImage(cv::Rect(85, 0, 265, 150)).clone();
            cv::imshow("Crop", objectImage);
            
            //cvtColor(objectImage, objectImage, cv::COLOR_BGR2HSV);

            // Images for thresholding:
            cv::Mat detectRed = objectImage.clone();
            cv::Mat detectBlue = objectImage.clone();
            cv::Mat detectYellow = objectImage.clone();

            //Thresholding colors:
            cv::inRange(detectRed, cv::Scalar(0, 0, 80), cv::Scalar(80, 60, 150), detectRed); // R�d
            cv::inRange(detectBlue, cv::Scalar(65, 25, 0), cv::Scalar(125, 85, 40), detectBlue); //Bl�
            cv::inRange(detectYellow, cv::Scalar(0, 80, 100), cv::Scalar(110, 180, 180), detectYellow); //Gr�n

            /* HSV
            // Images for thresholding:
            cv::Mat detectRed1 = objectImage.clone();
            cv::Mat detectRed2 = objectImage.clone();
            cv::Mat detectBlue = objectImage.clone();
            cv::Mat detectYellow = objectImage.clone();

            //Thresholding colors:
            cv::inRange(detectRed1, cv::Scalar(0, 200, 70), cv::Scalar(40, 250, 110), detectRed1); // R�d
            cv::inRange(detectRed2, cv::Scalar(200, 200, 70), cv::Scalar(255, 250, 110), detectRed2); // R�d
            cv::Mat1b detectRed = detectRed1 | detectRed2;
            cv::inRange(detectBlue, cv::Scalar(130, 200, 50), cv::Scalar(170, 255, 115), detectBlue); //Bl�
            cv::inRange(detectYellow, cv::Scalar(20, 150, 100), cv::Scalar(80, 255, 150), detectYellow); //Gr�n
            */

            // Morphology:
            morphology(detectRed, elem);
            morphology(detectBlue, elem);
            morphology(detectYellow, elem);

            // Displaying thresholds:
            cv::imshow("Red", detectRed);
            cv::imshow("Blue", detectBlue);
            cv::imshow("Yellow", detectYellow);
            
            // We update our masks:
            updateMasks(wasteObjects);

            // Finding waste objects in each channel:
            findWasteObjects(detectRed, timeStamp, "red", wasteObjects);
            findWasteObjects(detectBlue, timeStamp, "blue", wasteObjects);
            findWasteObjects(detectYellow, timeStamp, "yellow", wasteObjects);

            // Remove passed masks:
            removePassedObjects(wasteObjects);

            // Remove objects if they overla
            removeOverlapMasks(wasteObjects);

            // Debuggning only!!!! slows performance a lot:
            
            cv::Mat wasteObjectMasks = plotWasteObjects(wasteObjects);
            cv::imshow("TEST", wasteObjectMasks);


            /*if (wasteObjects.size() > 0){
                std::cout << "SIZE: " << wasteObjects.size() << std::endl;
            }*/
        }
    
        if (cv::waitKey(25) == 27) break;  // If ESC is pushed then break loop

        loop_rate.sleep();
    }

    return 0;
}