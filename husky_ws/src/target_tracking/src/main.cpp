#include <iostream>
#include <vector>

#include <ros/ros.h>

#include <image_transport/image_transport.h>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <cv_bridge/cv_bridge.h>

#include <axis_camera/Axis.h>

using namespace std;
using namespace cv;
using namespace ros;

double lex;
double ley;

double dex;
double dey;

double iex;
double iey;

double kpx = 0.01;
double kix = 0.001;
double kdx = 0.01;

double kpy = 0.01;
double kiy = 0.001;
double kdy = 0.01;

int thresh = 100;
int max_thresh = 255;
RNG rng(12345);

image_transport::Publisher image_pub;
ros::Publisher camera_ctl;
axis_camera::Axis cam_msg;

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

  camera_ctl = n.advertise<axis_camera::Axis>("axis/cmd", 1);

  cam_msg.pan = 0;
  cam_msg.tilt = 0;
  cam_msg.zoom = 0;
  cam_msg.brightness = 5000;
  cam_msg.autofocus = true;

  camera_ctl.publish(cam_msg);

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

void moveCamera(vector<Point> &corners) {
  if(corners.size() != 4) return;

  Point c = midpoint(corners[0], corners[2]);

  double ex = c.x - (1280.0 / 2.0);
  double ey = -(c.y - (720.0 / 2.0));

  iex += ex;
  dex = ex - lex;
  lex = ex;

  iey += ey;
  dey = ey - ley;
  ley = ey;

  double pan = ex * kpx + iex * kix + dex * kdx;
  double tilt = ey * kpy + iey * kiy + dey * kdy;

  cam_msg.pan = pan;
  cam_msg.tilt = tilt;

  cout << ex << " " << iex << " " << dex << " " << pan << endl;
  cout << ey << " " << iey << " " << dey<< " " << tilt << endl;

  camera_ctl.publish(cam_msg);
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

  //sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
  //image_pub.publish(msg);

  if(targets.size() != 0)
    moveCamera(targets[0]);
}

