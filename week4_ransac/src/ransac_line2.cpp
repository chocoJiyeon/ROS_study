#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <vector>

int num_of_edge = 300, num_of_it = 22, max_inlier_size = 50;
double tolerance = 4.0;
std::pair<int,int> line[2];
std::vector<std::pair<int,int>> edge;

void random_point(cv::Mat dataset)
{
    int r, c, x;
    for(int i =0; i<num_of_edge; i++)
    {
        c = rand()%500;
        if(i%5 ==1) x = rand()%100 - 50; else if (i%5 ==2) x = rand()%60 - 30; 
        else if(i%5 ==3) x = rand()%20 - 10; else x = rand()%4 -2;
        r = c/2 + 150 + x;
        if(dataset.at<uchar>(r,c))    i--;
        else  
        {
            dataset.at<uchar>(r,c) = 255; 
            edge.push_back(std::make_pair(r,c)); 
        }
    }
}

void mean_value(std::vector<std::pair<int,int>>* inlier, float* mean_r, float* mean_c)
{
    for(auto it : *inlier)
    {
        *mean_r += it.first;
        *mean_c += it.second;
    }
    *mean_r /= inlier->size();
    *mean_c /= inlier->size();
}

void save_line(std::vector<std::pair<int,int>>* inlier)
{
    float mean_r, mean_c, m, y, mh = 0, md = 0, rmse = 0;
    mean_value(inlier, &mean_r, &mean_c);
    for(auto it : *inlier)
    {
        mh += (it.first - mean_r)*(it.second - mean_c);
        md += (it.first - mean_r)*(it.first - mean_r);
    }
    m = mh/md;
    y = mean_c - m*mean_r;
    // line : r = m*c + y

}
void ransac()
{
    for(int i=0; i< num_of_it; i++)
    {
        std::vector<std::pair<int,int>> inlier;
        int rand_edge1, rand_edge2;
        int a1,a2,b1,b2;
        for(;;)
        {
            rand_edge1 = rand() % num_of_edge;
            rand_edge2 = rand() % num_of_edge;
            if(rand_edge1 != rand_edge2) break;
        }
        a1 = edge[rand_edge1].first;    b1 = edge[rand_edge1].second;
        a2 = edge[rand_edge2].first;    b2 = edge[rand_edge2].second;
        inlier.push_back(std::make_pair(a1,b1));
        inlier.push_back(std::make_pair(a2,b2));

        for(auto it : edge)
        {
            if((it.first == a1 && it.second == b1) || (it.first == a2 && it.second == b2)) break;
            int a,b;
            double distance;
            a = it.first; b = it.second;
            distance = abs((b2-b1)*a - (a2-a1)*b - a1*(b2-b1) + b1*(a2-a1))/std::sqrt(pow((b2-b1),2)+ pow((a2-a1),2));
            if(distance < tolerance)
            {
                inlier.push_back(std::make_pair(a,b));
            }   
        }
        if(inlier.size() > max_inlier_size)
        {
            save_line(&inlier);
            // max_inlier_size = cnt;
            // line[0].first = a1;     line[0].second = b1;
            // line[1].first = a2;     line[1].second = b2;
        }
    }
}//ransac()

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ransac_line");
    srand((unsigned int)time(NULL));

    int rows = 500, cols = 500;
    cv::Mat dataset(rows, cols, CV_8UC1);
    random_point(dataset);
    // cv::imshow("initial img",dataset);

    ransac();
    cv::Mat result_img;
    dataset.copyTo(result_img);
    float m = (float)(line[1].second - line[0].second)/(float)(line[1].first-line[0].first);
    cv::line(result_img, cv::Point(499, (int)((499-line[0].second)/m) +line[0].first), cv::Point(0, (int)(-line[0].second/m) +line[0].first), cv::Scalar::all(255));
    // cv::line(result_img, cv::Point(line[0].second, line[0].first), cv::Point(line[1].second, line[1].first), cv::Scalar::all(255));

    cv::imshow("result img",result_img);
    cv::waitKey(0); 
    return 0;
}