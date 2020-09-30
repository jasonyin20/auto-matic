#include <common/Logger.h>
#include <thread>
#include <mutex>
#include <deque>
#include <condition_variable>
#include <string>
#include <opencv2/opencv.hpp>
#include <driver/Camera.h>
#include <common/CycleQueue.h>
#include <sys/time.h>
#include <perception/ArmorDetection.h>
#include <system/AutomaticAimingSystem.h>
#include <common/Version.h>
#include <perception/RuneDetection.h>
#include <opencv2/dnn.hpp>
#include <common/Videotape.h>



std::string model_file = "/home/rm/桌面/Automatic aiming system/caffe/lenet_iter_200000.caffemodel";
std::string prototxt_file = "/home/rm/桌面/Automatic aiming system/caffe/lenet_train_test_deploy.prototxt";

CycleQueue<cv::Mat> cycleQueue(1);
Camera camera("bb");

void producer()
{
    if(!camera.open())
    {
        Log_Fatal <<"camera open fail";
    }
    while(1)
    {
        auto start = std::chrono::high_resolution_clock::now();
        cycleQueue.push(camera.getMat());
        auto end = std::chrono::high_resolution_clock::now();
        double time = (static_cast<std::chrono::duration<double,std::milli>>(end - start)).count();
        Log_Info <<"get time:" << time;
    }

}

void consumer()
{
//    while(1)
//    {
//        auto start = std::chrono::high_resolution_clock::now();
//      //  cv::Mat img = std::move(cycleQueue.pop());
//        auto end = std::chrono::high_resolution_clock::now();
//        if(img.empty())
//        {
//            Log_Info << "null";
//            continue;
//        }
//        double time = (static_cast<std::chrono::duration<double,std::milli>>(end - start)).count();
//        Log_Info <<"process time:" << time;
//       // cv::imshow("img", img);
//       // cv::waitKey(1);
//    }
}


using namespace cv;
using namespace std;
RNG g_rng(12345);//生成随机数，用于生成随机颜色
bool drawRect = false;
Point LeftPnt=Point(-1,-1);
Point mousePos= Point(-1, -1);
Mat cameraMatrix;
Mat distCoeffs;
//int flag=0;
void calAngle(Mat cam,Mat dis,int x,int y)
{

//    double fx=cam.at<double>(0,0);
//    double fy=cam.at<double>(1,1);
//    double cx=cam.at<double>(0,2);
//    double cy=cam.at<double>(1,2);
//    double k1=dis.at<double>(0);
//    double k2=dis.at<double>(1);
//    double p1=dis.at<double>(2);
//    double p2=dis.at<double>(3);
//    Point2f pnt;
//    vector<cv::Point2f>in;
//    vector<cv::Point2f>out;
//    in.push_back(Point2f(x,y));
//    //对像素点去畸变
//    undistortPoints(in,out,cam,dis,noArray(),cam);
//    pnt=out.front();
//    double rx=(pnt.x-cx)/fx;
//    double ry=(pnt.y-cy)/fy;
//
//    double tanx=(rx);
//    double tany=(ry);
//    cout <<out<<endl;
//
//    cout<< "xscreen: "<<x<<" xNew:"<<pnt.x<<endl;
//    cout<< "yscreen: "<<y<<" yNew:"<<pnt.y<<endl;
//    cout<< "angx: "<<atan((x-cx)/fx)/CV_PI*180<<" angleNew:"<<atan(rx)/CV_PI*180<<endl;
//    cout<< "angy: "<<atan((y-cy)/fy)/CV_PI*180<<" angleNew:"<<atan(ry)/CV_PI*180<<endl;
    double dx = x - 640;
    double dy = y - 512;
    double yaw = atan(dx / 1307) * 180 / 3.14159265459;
    double pitch = atan(dy / 1307) * 180 / 3.14159265459;
    Log_Info << "yaw :" << yaw;
    Log_Info << "pitch :" << pitch;
}

void on_mouse(int event, int x, int y, int flags, void *ustc)
//event鼠标事件代号，x,y鼠标坐标，flags拖拽和键盘操作的代号
{
//    flag=1;
    Mat& image = *(cv::Mat*) ustc;//这样就可以传递Mat信息了，很机智
    char temp[16];
    switch (event) {
        case EVENT_LBUTTONDOWN://按下左键
        {
            sprintf(temp, "(%d,%d)", x, y);
            putText(image, temp, Point(x, y), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 0, 255));
            drawRect = true;
            LeftPnt= Point(x, y);
            calAngle(cameraMatrix,distCoeffs,x,y);
        }	break;
        case EVENT_MOUSEMOVE://移动鼠标
        {
            mousePos = Point(x, y);
            if (drawRect)
            { }
        }break;
        case EVENT_LBUTTONUP:
        {
            drawRect = false;
            sprintf(temp, "(%d,%d)", x, y);
            putText(image, temp, Point(x, y), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 0, 255));
            //调用函数进行绘制
            cv::rectangle(image,LeftPnt, mousePos, cv::Scalar(g_rng.uniform(0, 255), g_rng.uniform(0, 255), g_rng.uniform(0, 255)));//随机颜色
        }break;
    }
}



/*int main(int argc, char *argv[])
{
    cv::VideoCapture cap;
    cap.open("/home/rm/桌面/RM2019能量机关视频（位于桥上拍摄）/机器人视角 蓝色 背景暗.avi");
    cv::Mat img;
    RuneDector dector;
    RuneLeaf leaf;
    dector.setRuneColor(RED);
    std::vector<cv::Point2f> shotPoints;
    int count = 0;
    double x,y,r;
    while(true)
    {
        Mat bin;
        cap >> img;
        dector.extractColor(img, bin);
        static const cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT,cv::Size(3,3));
        static const cv::Mat kernel1 = cv::getStructuringElement(cv::MORPH_RECT,cv::Size(9,9));
        cv::morphologyEx(bin, bin, cv::MORPH_OPEN,kernel);
        cv::dilate(bin, bin, kernel1);
        dector.findRuneLeaf(bin, leaf);
        if(dector.isSwitch(leaf))
        {
            count = 0;
        }
        dector._lastLeafPoint = leaf._shotPoint;
        if(count < 25)
        {
            shotPoints.emplace_back(std::move(dector._lastLeafPoint));
            count++;
        }
        if(count == 25)
        {
            dector.fittingCircle(shotPoints, x, y, r);
            count++;
        }
        if(count >25)
        {

            //Log_Info << "x: " << x << "y: " << y << "r: " << r;
            cv::circle(img, cv::Point(x,y), r, cv::Scalar(255,0,0), 3);
            double angle = dector.getTargetPolarAngle(x, y, leaf._shotPoint);
            std::string b = to_string(angle);
            if(dector.getRoationDirection(angle) == 1)
            {
                cv::Point2f centerPoint(x,y);
                cv::Point2f afterRotate =  dector.getRotatePoint(centerPoint,leaf._shotPoint, 30);
                cv::circle(img, afterRotate, 8, cv::Scalar(0, 255, 0));
            }

            cv::putText(img, b, leaf._shotPoint, 1, 2, cv::Scalar(255,0,255));
        }
        cv::circle(img, leaf._shotPoint, 2, cv::Scalar(0,255,0));
       // Log_Info <<"x: " <<leaf._shotPoint.x <<", " << "y: " << leaf._shotPoint.y;
        std::string a = to_string(leaf._leafContourArea/*/ /*leaf._leafRectArea);
        //cv::putText(img, a, leaf._shotPoint, 1, 2, cv::Scalar(255,0,255));
        imshow("img",img);
        imshow("show",bin);
        cv::waitKey(1);
    }
}*/


//int main(int argc, char** argv)
//{
//    FeedBackFrame feedBackFrame;
//    SerialPort serialPort("/dev/RMUSB");
//    serialPort.open(115200);
//    //while(true)
//    //{
//        bool flag = serialPort.receive();
//        feedBackFrame = serialPort.getFeedBack();
//      //  Log_Info << flag;
//        Log_Info <<(int) feedBackFrame.model;
//        Log_Info << (int)feedBackFrame.color;
//    //}
//    return 0;
//}




int main(int argc , char** argv)
{
    camera.open();
    ArmorDetector detector;
    SaveFrame _saveframe;
    detector.setEnemyColor(BLUE);
    cv::dnn::Net net;
    net = cv::dnn::readNetFromCaffe(prototxt_file,model_file);

    for (;;)
    {
        cv::Mat src = camera.getMat();
        cv::imshow("src",src);
        if(src.empty())
        {
            continue;
        }

        cv::Mat binnaryColor;
        cv::Mat binnaryBightness;
        detector.extractColor(src, binnaryColor, false);
        detector.extractBrightness(src, binnaryBightness);
        detector.findArmoredLights(binnaryColor, binnaryBightness);
        if (detector.findArmors(src))
        {
            Armor armor = detector.strikingDecision();
            cv::Point2f p[4];
            armor.armorRotatedRect.points(p);
            RotatedRect screen_rect = RotatedRect(armor.armorRotatedRect.center, Size2f(armor.armorRotatedRect.size.width * 0.9, armor.armorRotatedRect.size.height * 2.4),
                                                  armor.armorRotatedRect.angle);

            Size size = Point(src.cols, src.rows);
            Point p1, p2;
            int x = screen_rect.center.x - screen_rect.size.width / 2;
            int y = screen_rect.center.y - screen_rect.size.height / 2 ;
            p1 = Point(x, y);
            x = screen_rect.center.x + screen_rect.size.width / 2 ;
            y = screen_rect.center.y + screen_rect.size.height / 2;
            p2 = Point(x, y);
            Rect roi_rect =Rect(p1, p2);
            Mat input_sample;
            if(detector.makeRectSafe(roi_rect, size))
            {
                input_sample = src(roi_rect).clone();
                cv::imshow("roi",src);
                resize(input_sample, input_sample, Size(28, 28));
                _saveframe.savepicture(input_sample);
                cv::imshow("roi1",input_sample);
                cv::convertScaleAbs(input_sample, input_sample, 4);
                cvtColor(input_sample, input_sample,COLOR_BGR2GRAY);
                cv::imshow("roi2",input_sample);
                Mat inputBlob = dnn::blobFromImage(input_sample, 0.00390625f, Size(28, 28), Scalar(), false);
                Mat prob;
                net.setInput(inputBlob, "data");
                prob = net.forward("loss");

                int classId;
                double classProb;
                Mat probMat = prob.reshape(1, 1);
                Point classNumber;
                cv::minMaxLoc(probMat, NULL, &classProb, NULL, &classNumber);
                classId = classNumber.x;    // 类别：0是45度，1是装甲板
                cout << "class ID: " << classId << "\n\n";

            }
            detector._armorLights.clear();
            detector._armors.clear();
            char c = cv::waitKey(50);
            if (c ==27)
            {
                break;
            }
        }

    }
    return 0;
}





//int main(int argc, char** argv)
//{
////    camera.open();
////    ArmorDetector detector;
////    detector.setEnemyColor(BLUE);
////
////    for(;;)
////    {
////        cv::Mat src = camera.getMat();
////        if(src.empty())
////        {
////            continue;
////        }
////        cv::Mat binaryColor;
////        cv::Mat binaryBightness;
////        std::stringstream ss;
////        detector.extractColor(src, binaryColor, false);
////        detector.extractBrightness(src, binaryBightness);
////        detector.findArmoredLights(binaryColor, binaryBightness);
//////        for(auto light : detector._armorLights)
//////        {
//////            for(int i = 0; i < 4; ++i)
//////            {
//////                cv::line(src, light.rotateRectPoints[i], light.rotateRectPoints[(i+1)%4], cv::Scalar(0 , 255, 0), 3);
//////            }
//////
////////            ss << light.angle;
////////            cv::putText(src, ss.str(), light.rotateRectPoints[2],cv::FONT_HERSHEY_SIMPLEX,0.5,cv::Scalar(0,0,255), 0.1);
////////            ss.str("");
//////            ss << /*"x:" <<(int)light.center.x << " " <<*/ "y: " << (int)light.center.y ;
//////            cv::putText(src, ss.str(), light.center,cv::FONT_HERSHEY_SIMPLEX,0.5,cv::Scalar(0,255,0), 0.1);
//////            ss.str("");
//////            ss << "height: " << light.size.height;
//////            cv::putText(src, ss.str(), light.rotateRectPoints[2],cv::FONT_HERSHEY_SIMPLEX,0.5,cv::Scalar(0,255,0), 0.1);
//////            ss.str("");
//////        }
////        if(detector.findArmors(src))
///         {
////            Armor armor = detector.strikingDecision();
////
////            cv::Point2f p[4];
////            armor.armorRotatedRect.points(p);
////            for (int i = 0; i < 4; ++i)
/// q           {
////                ss << //(armor.armorRotatedRect.size.height > armor.armorRotatedRect.size.width ? \
////                        armor.armorRotatedRect.size.width : armor.armorRotatedRect.size.height)
////                      //  armor.armorRotatedRect.size.height * armor.armorRotatedRect.size.width;
////                      armor.armorRotatedRect.angle
////                       ;
////                cv::line(src, p[i], p[(i + 1) % 4], cv::Scalar(0, 255, 0), 3);
////                cv::putText(src, ss.str(), p[2], cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 0.1);
////                ss.str("");
////            }
////        }
////        cv::circle(src, cv::Point(640, 512), 2, cv::Scalar(0,255,0));
////        detector._armorLights.clear();
////        detector._armors.clear();
////        cv::imshow("color", binaryColor);
////        cv::imshow("bightness", binaryBightness);
////        cv::imshow("and", src);
////        cv::waitKey(1);
////    }
////    return 0;
////    SerialPort serialPort("/dev/RMUSB");
////    if(!serialPort.open(115200))
////    {
////        Log_Info << "fail";
////        return  0;
////    }
////    uint8_t  i = 0;
////    while(true)
////    {
//////        if (!serialPort.receive()){
//////            Log_Info << "fail";
//////            continue;
//////        }
//////        FeedBackFrame feedBackFrame = serialPort.getFeedBack();
//////        std::cout << "begin" << std::endl;
//////        std::cout<<std::hex <<(int)feedBackFrame.frameSeq << std::endl;
//////        std::cout<< std::hex <<(int)feedBackFrame.one << std::endl;
//////        std::cout<< std::hex << (int)feedBackFrame.two << std::endl;
//////        std::cout<< std::hex <<(int)feedBackFrame.three << std::endl;
//////        std::cout<< std::hex <<(int)feedBackFrame.four << std::endl;
//////        Log_Info << std::hex<<feedBackFrame.frameSeq;
//////        Log_Info << std::hex<<feedBackFrame.one;
//////        Log_Info << std::hex<<feedBackFrame.two;
//////        Log_Info << std::hex<<feedBackFrame.three;
//////        Log_Info << std::hex<<feedBackFrame.four;
////        ControlFrame controlFrame = {i++, 0, 1, 2};
////        serialPort.send(controlFrame);
////        cv::waitKey(5);
////    }
//    int16_t a = 256;
//    uint8_t b = a >> 8;
//      Log_Info << (int )b;
//    return  0;
//}



