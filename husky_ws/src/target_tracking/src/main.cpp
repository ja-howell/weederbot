#include <iostream>
#include <vector>

#include <ros/ros.h>

#include <image_transport/image_transport.h>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <cv_bridge/cv_bridge.h>

using namespace std;
using namespace cv;
using namespace ros;

int thresh = 100;
int max_thresh = 255;
RNG rng(12345);

image_transport::Publisher image_pub;

void targetDetect(Mat &);

void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
  try
  {
    Mat raw = cv_bridge::toCvShare(msg, "bgr8")->image;
    targetDetect(raw);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
  
}


int main(int argc, char **argv) {
  ROS_INFO("Hello world!");

  init(argc, argv, "target_tracker");
  NodeHandle n;

  image_transport::ImageTransport it(n);
  image_transport::Subscriber image_topic = it.subscribe("axis/image_raw_out", 1, imageCallback);
  image_pub = it.advertise("axis/image_marked", 1);

  spin();

  return 0;
}

float mag(Point a)
{
	return sqrt(a.x*a.x + a.y*a.y);
}

Point midpoint(Point a, Point b) {
  return Point((a.x + b.x)/ 2.0, (a.y+b.y)/2.0);
}

void targetDetect(Mat &image)
{
  Mat canny_output;

  /// Detect edges using canny
  Canny( image, canny_output, thresh, thresh*2, 3 );

  /// Find contours
  vector<vector<Point> > contours;
  vector<Vec4i> hierarchy;
  vector<vector<Point> > targets;
  findContours( canny_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_TC89_L1, Point(0, 0) );

  /// Draw contours
  for( int i = 0; i < contours.size(); i++ )
  {
    if(hierarchy[i][2]!=-1) continue;
    int depth = 0;
    int cont = i;
    while(hierarchy[cont][3] != -1)
    {
        cont = hierarchy[cont][3];
        depth++;
        if(depth == 5) break;
    }

    if(depth == 5)
    {
        Point2f cnt;
        float r;
        minEnclosingCircle(contours[cont], cnt, r);
        Point cen(cnt.x, cnt.y);

        // Order all of the points in this contour by distance to the center
        sort(contours[cont].begin(), contours[cont].end(), 
        [cen](const Point &a, const Point &b) -> bool 
        { return mag(cen-a) > mag(cen-b); });

        // Add the four most extreme points to points
        vector<Point> points;
        for(int n = 0; points.size() < 4 && n < contours[cont].size(); n++)
        {
            // Don't add this point if it is closer to another point then it is to the center
            bool tooClose = false;
            for(const Point &p : points)
            {
                if(mag(contours[cont][n] - p) < mag(contours[cont][n] - cen))
                    tooClose = true;
            }

            if(!tooClose)
            {
                circle(image, contours[cont][n], 1, Scalar(0,255,0), 2);
                points.emplace_back(contours[cont][n]);
            }
        }

        // Order the points in clockwise order
        sort(points.begin(), points.end(), 
        [cen](const Point &a, const Point &b) -> bool 
        { return atan2((cen-a).y, (cen-a).x) > atan2((cen-b).y, (cen-b).x); });

        targets.emplace_back(points);
        }
    }

  for(vector<Point> target : targets) {
    line(image, midpoint(target[0], target[1]), midpoint(target[2], target[3]), Scalar(0, 0, 255));
    line(image, midpoint(target[1], target[2]), midpoint(target[0], target[3]), Scalar(0, 0, 255));
  }


  sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
  image_pub.publish(msg);
}

