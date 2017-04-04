//
//  main.cpp
//  AR_Project_latest_version
//
//  Created by 欧阳佐坤 on 2017/3/20.
//  Copyright © 2017年 欧阳佐坤. All rights reserved.
//


#include <iostream>
#include <aruco/aruco.h>
#include <aruco/cvdrawingutils.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;
using namespace aruco;

//calculate perimeter
float perimeter(const vector<Point2f> &a)//求多边形周长。
{
    float sum=0,dx,dy;
    for(size_t i=0;i<a.size();i++)
    {
        size_t i2=(i+1) % a.size();
        
        dx = a[i].x - a[i2].x;
        dy = a[i].y - a[i2].y;
        
        sum += sqrt(dx*dx + dy*dy);
    }
    
    return sum;
}



int main() {
    VideoCapture capVideo(0);
    /*
     if (!capVideo.isOpened()) {
     cout << "Webcam is not open." << endl;
     }*/
    while (true) {
    Mat frame;
    //Mat frame = imread("/Users/ouyangzuokun/Desktop/marker9.jpg");
    capVideo.read(frame);
    
    Mat GRAYframe, BINframe;
    
    int min_size = 200; // Here the min_size of polygon is defined by hand (after trying several times)
    
    int thresh_size = (min_size/4)*2+1;
    
    cvtColor(frame, GRAYframe, CV_BGR2GRAY);
    adaptiveThreshold(GRAYframe, BINframe, 255, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY_INV, thresh_size, thresh_size/3);
    morphologyEx(BINframe, BINframe, MORPH_OPEN, Mat());
    
    
    vector<vector<Point>> all_contours;
    vector<vector<Point>> contours;
    findContours(BINframe, all_contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);
    
    for (int i = 0; i < all_contours.size(); ++i)
    {
        if (all_contours[i].size() > min_size)
        {
            contours.push_back(all_contours[i]);
        }
    }
    /*
    Mat img_dc = frame;
    
    for (int i = 0; i < contours.size(); i++) {
        drawContours(img_dc, contours, i, Scalar(0,0,255), 2, 8);
    }
    
    namedWindow("Contours");
    imshow("Contours", img_dc);
    */
    vector<Point2f> approx_poly; // similar shape
    vector<aruco::Marker> possible_markers; // possible markers
    
    // analyze every marker if it is a marker-liked polygon
    for (int i = 0; i < contours.size(); ++i)
    {
        //近似一个多边形
        double eps = contours[i].size()*0.05; // epsilon 近似的精度，原始曲线与近似曲线之间的最大距离
        //使多边形边缘平滑，得到近似的多边形 使用approxPolyDP()函数
        approxPolyDP(contours[i], approx_poly, eps, true);
        
        //这里只考虑四边形
        if (approx_poly.size() != 4)
            continue;
        
        //而且必须是凸面的
        if (!isContourConvex(approx_poly))
            continue;
        
        //确保相邻的两点间的距离“足够大”－大到是一条边而不是短线段就是了
        float min_side = FLT_MAX;
        for (int i = 0; i < 4; ++i)
        {
            Point side = approx_poly[i] - approx_poly[(i+1)%4];//这里应该是2维的相减
            min_side = min(min_size, side.dot(side));//求2维向量的点积，就是XxY，然后找出最小的距离
        }
        
        //确保距离不要太短
        if (min_side < 50)
            continue;
        
        
        
        
        
        
        //所有的测试通过了，保存标识候选，当四边形大小合适，则将该四边形maker放入possibleMarkers容器内
        //保存相似的标记
        Marker marker(approx_poly); //constructor Marker(const vector<Point2f> &corner)
        
        
        
        /*逆时针保存这些点
         //从代码推测，marker中的点集本来就两种序列：顺时针和逆时针，这里要把顺时针的序列改成逆时针，在多边形逼近时，多边形是闭合的，则不是顺时针就是逆时针
         //在第一个和第二个点之间跟踪出一条线，如果第三个点在右边，则点是逆时针保存的//逆时针排列这些点,第一个点和第二个点之间连一条线,如果第三个点在边，那么这些点就是逆时针*/
        
        //在approxPolyDP寻找多边形时，顶点摆放次序有逆时针和顺时针两种，我们希望这些顶点按照逆时针摆放
        
        
        
        
        Point2f v1 = marker[1] - marker[0];
        Point2f v2 = marker[2] - marker[0];
        
        
        
        /*行列式的几何意义是什么呢？有两个解释：一个解释是行列式就是行列式中的行或列向量所构成的超平行多面体的有向面积或有向体积；另一个解释是矩阵A的行列式detA就是线性变换A下的图形面积或体积的伸缩因子。
         //以行向量a=(a1,a2)，b=(b1,b2)为邻边的平行四边形的有向面积：若这个平行四边形是由向量沿逆时针方向转到b而得到的，面积取正值；若这个平行四边形是由向量a沿顺时针方向转到而得到的，面积取负值； */
        
        
        
        double o = (v1.x * v2.y) - (v1.y * v2.x);
        
        
        if(o<0.0) //如果第三个点在左边，那么交换第一个点和第三个点，逆时针保存
            swap(marker[1],marker[3]);
        possible_markers.push_back(marker);
        
    }
    
    
    
    
    
    vector<Marker> detectedMarkers;
    //移除角落太接近的元素
    //第一次检测相似性
    vector<pair<int, int>> tooNearCandidates;
    for (size_t i = 0; i < possible_markers.size(); i ++)
    {
        const Marker& m1 = possible_markers[i];
        //计算两个maker四边形之间的距离，四组点之间距离和的平均值，若平均值较小，则认为两个maker很相近,把这一对四边形放入移除队列。//计算每个边角到其他可能标记的最近边角的平均距离
        for (size_t j = i + 1; j < possible_markers.size(); j ++)
        {
            const Marker& m2 = possible_markers[j];
            float distSquared = 0.0;
            for (int c = 0; c < 4; c ++)
            {
                Point v = m1[c] - m2[c];
                //向量的点乘－》两点的距离
                distSquared += v.dot(v);
            }
            distSquared /= 4;
            if (distSquared < 100)
            {
                tooNearCandidates.push_back(pair<int, int>(i,j));
            }
        }
    }
    
    
    
    
    
    
    
    //移除了相邻的元素对的标识
    //计算距离相近的两个marker内部，四个点的距离和，将距离和较小的，在removlaMask内做标记，即不作为最终的detectedMarkers
    vector<bool> removalMask(possible_markers.size(),false);//创建Vector对象，并设置容量。第一个参数是容量，第二个是元素。
    for(size_t i=0;i<tooNearCandidates.size();i++)
    {
        //求这一对相邻四边形的周长
        
        float p1 = perimeter(possible_markers[tooNearCandidates[i].first]);
        float p2 = perimeter(possible_markers[tooNearCandidates[i].second]);
        
        //谁周长小，移除谁
        size_t removalIndex;
        if(p1 > p2)
            removalIndex = tooNearCandidates[i].second;
        else
            removalIndex = tooNearCandidates[i].first;
        
        removalMask[removalIndex] = true;
    }
    
    //返回候选，移除相邻四边形中周长较小的那个，放入待检测的四边形的队列中。//返回可能的对象
    detectedMarkers.clear();
    for(size_t i = 0;i<possible_markers.size();i++)
    {
        if(!removalMask[i])
            detectedMarkers.push_back(possible_markers[i]);
    }
    
    for (int i = 0; i < detectedMarkers.size(); i++){
        cout << "detectedMarkers[" << i << "] : " << detectedMarkers[i] << endl;
    }
    
    vector<Point2f> points;
    for(int  i = 0; i < detectedMarkers.size(); i++) {
        for (int j = 0; j < 4; j++) {
            points.push_back(detectedMarkers[i][j]);
        }
    }
    
    //cout << "points[0] : " << points[0] << endl;
    //cout << "detectedMarkers.size() : " << detectedMarkers.size() << endl;
    
    
    
    //cout << Markers.size() << endl;
    
    
    /*
    CameraParameters CP;
    CP.readFromXMLFile("/Users/Ouyangzuokun/Downloads/opencv_contrib-master/modules/aruco/samples/detector_params.yml");
    CP.resize(frame.size());
    */
    
    //for each marker, draw info and its boundaries in the image
    
    for (unsigned int i=0;i<detectedMarkers.size();i++) {
        //cout<<detectedMarkers[i]<<endl<<endl;
        detectedMarkers[i].draw(frame,Scalar(0,0,255),2);
        //CvDrawingUtils::draw3dAxis(frame, detectedMarkers[i], CP);
    }
    
    //bitMatrixRotate
    //
    //Dictionary dictionary = getPredefinedDictionary(PREDEFINED_DICTIONARY_NAME::DICT_6X6_250);
    
    //detectMarkers(frame, );
    
    
    /*
    aruco::MarkerDetector MarkerDetection;
    vector<Marker> detctedMarker123;
    MarkerDetection.detect(frame, detctedMarker123);
    for (unsigned int i=0;i<detctedMarker123.size();i++) {
        cout<<detctedMarker123[i]<<endl<<endl;
        detctedMarker123[i].draw(frame,Scalar(0,0,255),2);
    }
    */
    
    
    
    
    
    
    
    imshow("Original", frame);
    //imshow("Gray", GRAYframe);
    imshow("Binary", BINframe);
    waitKey(1);
    }
    
    return 0;
}

