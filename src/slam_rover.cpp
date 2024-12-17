#include <chrono>
#include <iostream>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>
#include <NumCpp.hpp>



class SLAM_RoverNode : public rclcpp::Node {

public:
  SLAM_RoverNode():Node("slam_rover_node") {
    // int codec = cv::VideoWriter::fourcc('M','J','P','G');
    cv::Size frame_size(640, 480);


    vid_subscriber = this->create_subscription<sensor_msgs::msg::Image>(
                  "/camera_scan_fwd/image_raw", 20, std::bind(&SLAM_RoverNode::image_process_callback, 
                  this,std::placeholders::_1));

    publisher_vel = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel",40);
    timer_vel_pub = this->create_wall_timer(std::chrono::microseconds(1000),
                std::bind(&SLAM_RoverNode::send_cmd_vel, this));


  }

private:
  rclcpp::Time current_time;
  rclcpp::Time prev_time;
  float prev_error = 0.0f; // Pid prev error

  std::string action = "Blank";
  float velocity_lin, velocity_ang;

  int16_t mid_point_frame;
  int16_t mid_point_edge = 0; 
  int16_t mid_point_lin_vel_edge = 0; 
  int16_t last_pt_caution;

  int r1= 300, c1 = 0;
  int row_of_interest = 120;
  int row_of_lin_vel = row_of_interest - 10;
  int row_of_caution = 04;


  float mid_pt_error = 0.0f;
  float mid_pt_lin_vel_error = 0.0f;

  std::vector<int16_t> prev_edge;
  std::vector<int16_t> prev_edge_lin_vel;

  void image_process_callback(const sensor_msgs::msg::Image::SharedPtr msg_vid) {

    cv::Mat vid_output;
    cv::Mat resized_frame;
    // nc::NdArray<double> edge;


    cv::Mat frame = cv_bridge::toCvCopy(msg_vid, "bgr8")->image;

    // // Resize frame to 640x480
    cv::resize(frame, resized_frame, cv::Size(640, 480));


    // ###############  --------------Segmentation-------------------------------
    uint8_t light = 110 ;
    uint8_t dark = 255 ;

    // ## Defining the color Upper bound and Lower bound limits to extract 
    cv::Scalar light_line(light,light,light);
    cv::Scalar dark_line(dark,dark,dark);

    cv::Mat mask ;
    cv::inRange(resized_frame, light_line, dark_line, mask);

    // cv::Mat gray;
    // cv::cvtColor(resized_frame, gray, cv::COLOR_RGB2GRAY);

    //-----------------------------# Boundaries Extraction----------------------
    //## applying the canny edge detector function
    cv::Mat canny;
    cv::Canny(mask,canny,50,150);

    // int r1= 300, c1 = 0;
    // int row_of_interest = 120;
    // int row_of_lin_vel = row_of_interest - 10;
    // int row_of_caution = 04;

    std::vector<int16_t> edge{};
    std::vector<int16_t> edge_lin_vel{};
    std::vector<int16_t> edge_of_caution{};
    edge_of_caution.clear();

    cv::Mat cropped_canny = canny(cv::Range(r1, canny.rows), cv::Range(c1,canny.cols));

    uint8_t pixel_value;
    uint8_t pixel_value_lin_vel;
    uint8_t pixel_value_caution;


/// Filling up Edge with 2 points from 2 edges............//////
    for (int col= c1; col < cropped_canny.cols; col++){
      pixel_value = cropped_canny.at<uint8_t>(row_of_interest, col); 
      pixel_value_caution = cropped_canny.at<uint8_t>(row_of_caution, col); 
      pixel_value_lin_vel = cropped_canny.at<uint8_t>(row_of_lin_vel, col) ;

      if (pixel_value_caution == 255){
        last_pt_caution = col;      
        edge_of_caution.push_back(col);
      }


      if(pixel_value ==255){
        if(!edge.empty()){
          if ( edge.size() < 2) { 
                  // RCLCPP_INFO(this->get_logger(), "column: %d added", col);
            if(abs(edge.back()- col) < 5){
              edge.pop_back();
            }
            edge.push_back(col);
          }
        }
        else{
          edge.push_back(col);
        }    
      } 
      
    }
    
    if (edge.size() < 2 || abs(edge[0]) < 0 || abs(edge[1]) > cropped_canny.cols){
      action = "Tracking Dummy (prev edge)";
      edge = prev_edge;
      // velocity_lin = 0.0;
    }
    prev_edge = edge;

    //// Processing mid points....................////
    // float left_edge = static_cast<float>(edge[0]);
    // float right_edge= static_cast<float>(edge[1]);

    int16_t left_edge = edge[0];
    int16_t right_edge= edge[1];

    mid_point_edge = left_edge+(right_edge- left_edge)/2 ;
    // RCLCPP_INFO(this->get_logger(), "mid_point of edges== %d", mid_point_edge);

    if(edge_of_caution.size() < 1){
      if(last_pt_caution > mid_point_frame){
        mid_point_edge += (last_pt_caution - mid_point_frame)/1;
        action = "Caution- Take Right";
      }
      else{
        action = "Caution- Take Left";}
        mid_point_edge += (last_pt_caution - mid_point_frame)/1;
    }

    mid_point_lin_vel_edge = mid_point_edge + 4;


    mid_point_frame = cropped_canny.cols/2;

    cropped_canny.at<uint8_t>(row_of_caution, last_pt_caution-1) = 255;
    cropped_canny.at<uint8_t>(row_of_caution, last_pt_caution-2) = 255;
    cropped_canny.at<uint8_t>(row_of_caution, last_pt_caution-3) = 255;
    cropped_canny.at<uint8_t>(row_of_caution, last_pt_caution-4) = 255;
    cropped_canny.at<uint8_t>(row_of_caution, last_pt_caution-+1) = 255;

    cropped_canny.at<uint8_t>(row_of_interest, mid_point_edge) = 255;
    cropped_canny.at<uint8_t>(row_of_interest-1, mid_point_frame) = 255;
    cropped_canny.at<uint8_t>(row_of_interest, mid_point_frame) = 255;
    cropped_canny.at<uint8_t>(row_of_interest+1, mid_point_frame) = 255;

    cropped_canny.at<uint8_t>(row_of_lin_vel, mid_point_lin_vel_edge) = 255;

    RCLCPP_INFO(this->get_logger(), "Edges == %d, %d !! Mid_Edge == %d", edge[0], edge[1] , mid_point_edge);
    // RCLCPP_INFO(this->get_logger(), "frame size-- %dx%d", cropped_canny.cols,cropped_canny.rows);

    // mid_pt_error = mid_point_frame - mid_point_edge;

    cv::putText(cropped_canny, action, 
                cv::Point(20, 80), 
                cv::FONT_HERSHEY_DUPLEX, 
                1.0, CV_RGB(255, 255, 255), 2);

    cv::putText(cropped_canny, std::to_string(velocity_lin), 
                cv::Point(20, 120), 
                cv::FONT_HERSHEY_DUPLEX, 
                1.0, CV_RGB(255, 255, 255), 2);

    vid_output = cropped_canny;


    cv::imshow("output", vid_output);
    if (cv::waitKey(1) == 27) {
      std::cout << "esc key is pressed by user" << std::endl;
    }
  }


  float PID_control(float ref_point, float current_state, float kp,float ki,float kd){

    current_time = this->now();
    float dt = (prev_time.nanoseconds()>0) ? (current_time - prev_time).seconds() : .001 ;
    prev_time = current_time;
    RCLCPP_INFO(this->get_logger(), "dt =>>  %f", dt);

    // float prev_error;

    float error = current_state - ref_point;
    float error_der = (error - prev_error)/dt;

    RCLCPP_INFO(this->get_logger(), "Error => %f, Previous Error => %f, Error_d => %f", error, prev_error, error_der);

    float feedback = kp*error + kd*error_der;
    // prev_error = (abs(error) > 0) ? (error) : 0.0;
    prev_error = error;

    return feedback;
  }


  void follow_track(){
    // current_time = this->now();
      
    // if (velocity_lin == 0 && current_time.nanoseconds() == 0.0f){
    //   velocity_lin = 0.65;
    // }

    float control_output_ang_vel = PID_control(mid_point_edge, mid_point_frame, 0.10f, 0.540f, 0.0f);
    velocity_ang = std::clamp(control_output_ang_vel, -1.57f, 1.57f);

    float control_output_lin_vel = PID_control(row_of_lin_vel, row_of_interest, 0.10f, 0.540f, 0.0f);
    velocity_lin = std::clamp(control_output_lin_vel, -0.7f, 0.7f);

    if(action == "Caution- Take Right"){
      velocity_lin = 0.0;
      velocity_ang = -1.5;
    }
    
    else if(action == "Caution- Take Left" ){
      velocity_lin = 0.0;
      velocity_ang = 1.5;
    }

    else if(control_output_ang_vel == 0.0f) {
      action = "Going Straight";
      // velocity_lin = 0.65;
    }

    else if (control_output_ang_vel < 0.0f){
      action = "Steering Right";
      // velocity_lin = 0.55;
    }
    else if (control_output_ang_vel > 0.0f){
      action = "Steering Left";
      // velocity_lin = 0.55;
    }

    // else if(action == "Tracking Dummy (prev edge)"){
    //   velocity_lin = -0.30;
    // }


  }

  void send_cmd_vel(){
    // velocity_lin = 0.3f;

    follow_track();
    geometry_msgs::msg::Twist velocity_cmd;

    velocity_cmd.linear.x = velocity_lin;
    velocity_cmd.angular.z = velocity_ang;
    // RCLCPP_INFO(this->get_logger(), "-------Fwd Speed : %.2f", velocity_lin);
    
    publisher_vel-> publish(velocity_cmd);
  }

  // cv::VideoWriter video_writer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_vel;
  rclcpp::TimerBase::SharedPtr timer_vel_pub;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr vid_subscriber;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SLAM_RoverNode>());
  rclcpp::shutdown();
  return 0;
}