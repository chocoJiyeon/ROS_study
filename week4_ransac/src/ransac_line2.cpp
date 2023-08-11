#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <vector>

int num_of_edge = 300, num_of_it = 22, max_inlier_size = 50;
double tolerance = 4.0;
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

float cal_rmse(std::vector<std::pair<int,int>>* inlier, float* m, float* n)
{
    float rmse = 0;
    for(auto it : *inlier)
    {
        rmse += abs(*m*it.first - it.second + *n)/std::sqrt(pow(*m,2)+ pow(*n,2));
    } 
    rmse /= inlier->size();
    return std::sqrt(rmse);
}

void save_line(std::vector<std::pair<int,int>>* inlier, std::vector<float*>* line)
{
    float mean_r, mean_c, mh = 0, md = 0;
    float *line_info = new float[3];
    mean_value(inlier, &mean_r, &mean_c);
    for(auto it : *inlier)
    {
        mh += (it.first - mean_r)*(it.second - mean_c);
        md += (it.first - mean_r)*(it.first - mean_r);
    }
    line_info[0] = mh/md;
    line_info[1] = mean_c - line_info[0]*mean_r;
    // line : c = m*r + n
    line_info[2] = cal_rmse(inlier, &line_info[0], &line_info[1]);
    line->push_back(line_info);
}
void ransac(cv::Mat* result_img)
{
    std::vector<float*> line;
    int min_error = 999, min_it;
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
            save_line(&inlier, &line);
        }
    }
    for(int i =0; i< line.size(); i++)
    {
        if(line.at(i)[2] < min_error)
        {
            min_error = line.at(i)[2];
            min_it = i;
        }
    }
    cv::line(*result_img, 
                cv::Point(499,(int)((499-line.at(min_it)[1])/line.at(min_it)[0])), 
                cv::Point(0,(int)(-line.at(min_it)[1]/line.at(min_it)[0])), 
                cv::Scalar::all(255));

}//ransac()

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ransac_line");
    srand((unsigned int)time(NULL));

    int rows = 500, cols = 500;
    cv::Mat dataset(rows, cols, CV_8UC1);
    random_point(dataset);
    // cv::imshow("initial img",dataset);

    cv::Mat result_img;
    dataset.copyTo(result_img);
    ransac(&result_img);

    cv::imshow("result img",result_img);
    cv::waitKey(0); 
    return 0;
}