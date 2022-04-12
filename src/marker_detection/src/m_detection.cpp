#include<iostream>
#include<opencv4/opencv2/aruco.hpp>
#include<opencv4/opencv2/highgui.hpp>
#include<opencv4/opencv2/opencv.hpp>

// int main(){
//     std::cout << "m_detection" << std::endl;

//     cv::Mat input_img = cv::imread("/home/minji/marker/src/marker_detection/src/marker.jpg");
//     cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
//     cv::Mat markerImage;
//     while(!input_img.empty())
//     {
        
//         input_img.copyTo(markerImage);
//         std::vector<int> ids;
//         std::vector<std::vector<cv::Point2f> > corners;
//         cv::aruco::detectMarkers(input_img, dictionary, corners, ids);

//         if(ids.size() > 0)
//         {
//             cv::aruco::drawDetectedMarkers(markerImage, corners, ids);
//         }
//         cv::imshow("a", markerImage);
//         cv::waitKey(0);
//     }
//     //cv::Mat a = cv::imread("/home/minji/marker/src/marker_detection/src/lenna.png");
//     //cv::imshow("a", a);

    
//     // cv::aruco::drawMarker(dictionary, 61, 200, markerImage, 1);
//     // cv::imwrite("/home/minji/marker/src/marker_detection/src/marker61.png", markerImage);

    
//     return 0;
// }

int main(int arc, char** arv)
{
    cv::VideoCapture cap(0);

    if(!cap.isOpened())
    {
        printf("Can't open the camera");
        return -1;
    }

    cv::Mat img;

    while(1)
    {
        cap >> img;
        cv::imshow("camera+img", img);

        if(cv::waitKey(1)==27)
            break;
    }

    return 0;
}