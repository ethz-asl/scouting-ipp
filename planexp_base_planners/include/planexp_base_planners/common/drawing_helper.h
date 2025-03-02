//
// Created by Friedrich M. Rockenbauer on 01.11.24.
//

#ifndef PLANNER_PLANEXP_DRAWING_HELPER2_H
#define PLANNER_PLANEXP_DRAWING_HELPER2_H

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#define CV_RED    cv::Vec3b(   0,   0, 255 )
#define CV_GREEN  cv::Vec3b(   0, 255,   0 )
#define CV_BLUE   cv::Vec3b( 255,   0,   0 )
#define CV_BLACK  cv::Vec3b(   0,   0,   0 )
#define CV_GRAY   cv::Vec3b( 127, 127, 127 )
#define CV_WHITE  cv::Vec3b( 255, 255, 255 )
#define CV_YELLOW cv::Vec3b(   0, 255, 255 )
#define CV_CYAN   cv::Vec3b( 255, 255,   0 )
#define CV_PURPLE cv::Vec3b( 240,  32, 160 )


void draw_path_segment( cv::Mat img, cv::Point start, cv::Point end )
{
  int thickness = 2;
  int lineType = cv::LINE_8;
  line( img,
        start,
        end,
        CV_PURPLE,
        thickness,
        lineType );
}

void draw_tree_segment( cv::Mat img, cv::Point start, cv::Point end )
{
  int thickness = 1;
  int lineType = cv::LINE_8;
  line( img,
        start,
        end,
        CV_CYAN,
        thickness,
        lineType );
}

void draw_circle(cv::Mat& img, cv::Point location, cv::Scalar fill_color, cv::Scalar line_color, int radius) {
  circle( img,
          location,
          radius,
          fill_color,
          cv::FILLED,
          cv::LINE_AA );
  circle( img,
          location,
          radius,
          line_color,
          2,
          cv::LINE_AA );  
}

void draw_start( cv::Mat& img, cv::Point start )
{
  // draw_circle(img, start, CV_GREEN, CV_WHITE, 400/32); 
  cv::drawMarker(img, start, CV_BACK, cv::MarkerTypes::MARKER_DIAMOND, 10, 2, cv::LINE_AA);
}
void draw_goal( cv::Mat& img, cv::Point goal )
{
  // draw_circle(img, goal, CV_RED, CV_WHITE, 400/32); 
  cv::drawMarker(img, goal, CV_BACK, cv::MarkerTypes::MARKER_TILTED_CROSS, 10, 2, cv::LINE_AA);
}

#endif //PLANNER_PLANEXP_DRAWING_HELPER2_H
