#include <src/ardrone.h>
#include <opencv2/stitching/stitcher.hpp>
#define UNKNOWN_FLOW_THRESH 1e9


void makecolorwheel(vector<Scalar> &colorwheel)
  {
      int RY = 15;
      int YG = 6;
      int GC = 4;
      int CB = 11;
      int BM = 13;
      int MR = 6;

      int i;

      for (i = 0; i < RY; i++) colorwheel.push_back(Scalar(255,       255*i/RY,     0));
      for (i = 0; i < YG; i++) colorwheel.push_back(Scalar(255-255*i/YG, 255,       0));
      for (i = 0; i < GC; i++) colorwheel.push_back(Scalar(0,         255,      255*i/GC));
      for (i = 0; i < CB; i++) colorwheel.push_back(Scalar(0,         255-255*i/CB, 255));
      for (i = 0; i < BM; i++) colorwheel.push_back(Scalar(255*i/BM,      0,        255));
      for (i = 0; i < MR; i++) colorwheel.push_back(Scalar(255,       0,        255-255*i/MR));
  }

  void motionToColor(Mat flow, Mat &color)
  {
      if (color.empty())
          color.create(flow.rows, flow.cols, CV_8UC3);

      static vector<Scalar> colorwheel; //Scalar r,g,b
      if (colorwheel.empty())
          makecolorwheel(colorwheel);

      // determine motion range:
      float maxrad = -1;

      // Find max flow to normalize fx and fy
      for (int i= 0; i < flow.rows; ++i)
      {
          for (int j = 0; j < flow.cols; ++j)
          {
              Vec2f flow_at_point = flow.at<Vec2f>(i, j);
              float fx = flow_at_point[0];
              float fy = flow_at_point[1];
              if ((fabs(fx) >  UNKNOWN_FLOW_THRESH) || (fabs(fy) >  UNKNOWN_FLOW_THRESH))
                  continue;
              float rad = sqrt(fx * fx + fy * fy);
              maxrad = maxrad > rad ? maxrad : rad;
          }
      }

      for (int i= 0; i < flow.rows; ++i)
      {
          for (int j = 0; j < flow.cols; ++j)
          {
              uchar *data = color.data + color.step[0] * i + color.step[1] * j;
              Vec2f flow_at_point = flow.at<Vec2f>(i, j);

              float fx = flow_at_point[0] / maxrad;
              float fy = flow_at_point[1] / maxrad;
              if ((fabs(fx) >  UNKNOWN_FLOW_THRESH) || (fabs(fy) >  UNKNOWN_FLOW_THRESH))
              {
                  data[0] = data[1] = data[2] = 0;
                  continue;
              }
              float rad = sqrt(fx * fx + fy * fy);

              float angle = atan2(-fy, -fx) / CV_PI;
              float fk = (angle + 1.0) / 2.0 * (colorwheel.size()-1);
              int k0 = (int)fk;
              int k1 = (k0 + 1) % colorwheel.size();
              float f = fk - k0;
              //f = 0; // uncomment to see original color wheel

              for (int b = 0; b < 3; b++)
              {
                  float col0 = colorwheel[k0][b] / 255.0;
                  float col1 = colorwheel[k1][b] / 255.0;
                  float col = (1 - f) * col0 + f * col1;
                  if (rad <= 1)
                      col = 1 - rad * (1 - col); // increase saturation with radius
                  else
                      col *= .75; // out of range
                  data[2 - b] = (int)(255.0 * col);
              }
          }
      }
  }

void motionToGray(Mat flow, Mat &color)
{
  if (color.empty())
      color.create(flow.rows, flow.cols, CV_8UC1);

  // determine motion range:
  float maxrad = -1;

  // Find max flow to normalize fx and fy
  for (int i= 0; i < flow.rows; ++i)
  {
      for (int j = 0; j < flow.cols; ++j)
      {

          Vec2f flow_at_point = flow.at<Vec2f>(i, j);
          float fx = flow_at_point[0];
          float fy = flow_at_point[1];
          float r = sqrt(pow(i - flow.rows/2,2) + pow(j + flow.cols/2 , 2));
          if ((fabs(fx) >  UNKNOWN_FLOW_THRESH) || (fabs(fy) >  UNKNOWN_FLOW_THRESH))
              continue;
          float rad = sqrt(fx * fx + fy * fy) == 0 ? 0 : sqrt(fx * fx + fy * fy) / r;
          maxrad = maxrad > rad ? maxrad : rad;
      }
  }
  for (int i= 0; i < flow.rows; ++i)
  {
      for (int j = 0; j < flow.cols; ++j)
      {
          uchar *data = color.data + color.step[0] * i + color.step[1] * j;

          Vec2f flow_at_point = flow.at<Vec2f>(i, j);
          float r = sqrt(pow(i - flow.rows/2 , 2) + pow(j + flow.cols/2 , 2));

          float fx = flow_at_point[0] / maxrad;
          float fy = flow_at_point[1] / maxrad;
          if ((fabs(fx) >  UNKNOWN_FLOW_THRESH) || (fabs(fy) >  UNKNOWN_FLOW_THRESH))
          {
              data[0] = data[1] = data[2] = 0;
              continue;
          }

          float rad = sqrt(fx * fx + fy * fy) == 0 ? 0: sqrt(fx * fx + fy * fy) / r;
          float col = rad / maxrad * 255.0;
          //cout << float(0)/float(0) << endl;



              data[0] = (int)(col);

      }
  }
}
void  my_ardrone_node::process(const sensor_msgs::ImageConstPtr& cam_image){
    cv_bridge::CvImagePtr cv_ptr;
    vector<Mat> imgs;

    try
    {
      cv_ptr = cv_bridge::toCvCopy(cam_image,sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e){
      ROS_ERROR("cv_bridge exception:%s",e.what());

      return;
    }
    current_image = cv_ptr->image;
    Mat gray_current_image,flow,cflow,range_flow;
    cvtColor(current_image,gray_current_image,CV_BGR2GRAY);
    if(last_image.data != NULL)
    {
        calcOpticalFlowFarneback(last_image,gray_current_image,flow,0.5, 3,15, 3, 5, 1.2, 0);
        motionToGray(flow,cflow);
        //cvtColor(flow,cflow,CV_BGR2GRAY);
        inRange(cflow,220,255,range_flow);
        imshow("FBFLOW",cflow);
        cout << cflow.cols << cflow.rows << endl;
        imshow("RangeFlow",range_flow);
        imgs.push_back(current_image);
        imgs.push_back(cflow);
        imgs.push_back(range_flow);

        if(vm_flag)
        {
            vm_cflow << cflow;
            vm_range_flow << range_flow;
            vm_rgb << current_image;
        }
    }


    imshow(WINDOW,current_image);
    last_image = gray_current_image;
    int k = waitKey(1);
    if(k != -1)
        cout << k << endl;


    if(k == SPACE)//space for takeoff
    {
        takeoff();
    }
    if(k == ENTER)//enter for land
        land();
    if(k == UP || k == W_KEY)//up for forward
        forward(1);
    if(k == DOWN || k == S_KEY)//down for back
        back(1);
    if(k == LEFT || k == A_KEY)//left for turn_left
        turn_left(1);
    if(k == RIGHT || k == D_KEY)//right for turn_right
        turn_right(1);
    if(k == Q_KEY)
        left(1);
    if(k == E_KEY)
        right(1);
    if(k == R_KEY)
        vm_flag = true;





    return;

    }
