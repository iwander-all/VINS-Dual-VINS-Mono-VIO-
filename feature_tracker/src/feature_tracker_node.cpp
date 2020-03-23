#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Bool.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <thread>
#include <mutex>

#include "../include/feature_tracker.h"

#define SHOW_UNDISTORTION 0

vector<uchar> r_status;
vector<float> r_err;
queue<sensor_msgs::ImageConstPtr> img_buf;
queue<sensor_msgs::ImageConstPtr> img1_buf;
std::mutex m_buf;

ros::Publisher pub_img,pub_match;
ros::Publisher pub_restart;

FeatureTracker trackerData;

double first_image_time;
int pub_count = 1;
bool first_image_flag = true;
double last_image_time = 0;
bool init_pub = 0;

sensor_msgs::ImageConstPtr img_msg;
sensor_msgs::ImageConstPtr img1_msg;


void img_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
    //ROS_INFO("this is img_callback of feature tracker");
    m_buf.lock();
    img_buf.push(img_msg);
    m_buf.unlock();
}

void img1_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
    //ROS_INFO("this is img_callback of feature tracker");
    m_buf.lock();
    img1_buf.push(img_msg);
    m_buf.unlock();
}

cv_bridge::CvImageConstPtr getPtrFromImage(const sensor_msgs::ImageConstPtr &img_msg)
{
    cv_bridge::CvImageConstPtr ptr;
    if (img_msg->encoding == "8UC1")
    {
        sensor_msgs::Image img;
        img.header = img_msg->header;
        img.height = img_msg->height;
        img.width = img_msg->width;
        img.is_bigendian = img_msg->is_bigendian;
        img.step = img_msg->step;
        img.data = img_msg->data;
        img.encoding = "mono8";
        ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
    }
    else
        ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);
    return ptr;
}

void pubTrackImage(const cv::Mat &imgTrack, const double t)
{
    std_msgs::Header header;
    header.frame_id = "world";
    header.stamp = ros::Time(t);
    sensor_msgs::ImagePtr imgTrackMsg = cv_bridge::CvImage(header, "bgr8", imgTrack).toImageMsg();
    pub_match.publish(imgTrackMsg);
}

void f_process()
{
    while (true)
    {
        TicToc t_r;
        //ROS_INFO("this is feature tracking");
        while (true)
        {
            if (!img_buf.empty() && !img1_buf.empty())
                break;
            std::chrono::milliseconds dura(5);
            std::this_thread::sleep_for(dura);
        }

        m_buf.lock();
        double time0 = img_buf.front()->header.stamp.toSec();
        double time1 = img1_buf.front()->header.stamp.toSec();
        if (time0 < time1 - 0.003)
        {
            img_buf.pop();
            continue;
        }
        if (time0 > time1 + 0.003)
        {
            img1_buf.pop();
            continue;
        }
        double time = img_buf.front()->header.stamp.toSec();
        img_msg = img_buf.front();
        img1_msg = img_buf.front();
        //ROS_INFO("get a image frame");
        img_buf.pop();
        img1_buf.pop();
        m_buf.unlock();

        // change the format of the image
        cv_bridge::CvImageConstPtr ptr = getPtrFromImage(img_msg);
        cv_bridge::CvImageConstPtr ptr1 = getPtrFromImage(img1_msg);

        //TODO check
        if(ptr->image.empty() || ptr1->image.empty())
        {
            continue;
        }

        // read image and track features
        cv::Mat show_img = ptr->image;

        //extract feature publisher parts function
        trackerData.readImage(ptr->image.rowRange(0, ROW), 
                              ptr1->image.rowRange(0,ROW),img_msg->header.stamp.toSec());

        // renew global feature ids
        for (unsigned int i = 0;; i++)
        {
            bool completed = false;
            completed |= trackerData.updateID(i);

            if (!completed)
                break;
        }

        // publish features
        if (PUB_THIS_FRAME)
        {
            pub_count++;
            sensor_msgs::PointCloudPtr feature_points(new sensor_msgs::PointCloud);
            sensor_msgs::ChannelFloat32 id_of_point;
            sensor_msgs::ChannelFloat32 u_of_point;
            sensor_msgs::ChannelFloat32 v_of_point;
            sensor_msgs::ChannelFloat32 velocity_x_of_point;
            sensor_msgs::ChannelFloat32 velocity_y_of_point;
            sensor_msgs::ChannelFloat32 id_of_camera;

            feature_points->header = img_msg->header;
            feature_points->header.frame_id = "world";
            
            auto &un_pts = trackerData.cur_un_pts;
            auto &cur_pts = trackerData.cur_pts;
            auto &ids = trackerData.ids;
            auto &pts_velocity = trackerData.pts_velocity;

            unsigned int count_feature_left = 0;

            auto &un_pts1 = trackerData.cur_un_pts1;
            auto &cur_pts1 = trackerData.cur_pts1;
            auto &ids1 = trackerData.ids1;
            //auto &pts_velocity1 = trackerData.pts_velocity;

            unsigned int count_feature_right = 0;

            for (unsigned int j = 0; j < ids.size(); j++)
            {
                if (trackerData.track_cnt[j] > 1)
                {
                    geometry_msgs::Point32 p;
                    p.x = un_pts[j].x;
                    p.y = un_pts[j].y;
                    p.z = 1;

                    feature_points->points.push_back(p);
                    id_of_point.values.push_back(ids[j]);
                    u_of_point.values.push_back(cur_pts[j].x);
                    v_of_point.values.push_back(cur_pts[j].y);
                    velocity_x_of_point.values.push_back(pts_velocity[j].x);
                    velocity_y_of_point.values.push_back(pts_velocity[j].y);
                    id_of_camera.values.push_back(0);
                    count_feature_left++;
                }
            }

            // assert(count_feature_left == ids1.size());
            // assert(un_pts1.size() == ids1.size());
            // assert(cur_pts1.size() == ids1.size());

            for (unsigned int j = 0; j < ids1.size(); j++)
            {
                if (trackerData.track_cnt[j] > 1)
                {
                    geometry_msgs::Point32 p;
                    p.x = un_pts1[j].x;
                    p.y = un_pts1[j].y;
                    p.z = 1;

                    feature_points->points.push_back(p);
                    id_of_point.values.push_back(ids1[j]);
                    u_of_point.values.push_back(cur_pts1[j].x);
                    v_of_point.values.push_back(cur_pts1[j].y);
                    velocity_x_of_point.values.push_back(0);
                    velocity_y_of_point.values.push_back(0);
                    id_of_camera.values.push_back(1);

                    count_feature_right++;
                }
            }
            // assert(count_feature_left == count_feature_right);
            

            feature_points->channels.push_back(id_of_point);
            feature_points->channels.push_back(u_of_point);
            feature_points->channels.push_back(v_of_point);
            feature_points->channels.push_back(velocity_x_of_point);
            feature_points->channels.push_back(velocity_y_of_point);
            feature_points->channels.push_back(id_of_camera);
            ROS_DEBUG("publish %f, at %f", feature_points->header.stamp.toSec(), ros::Time::now().toSec());
            
            // skip the first image; since no optical speed on frist image
            if (!init_pub)
                init_pub = 1;
            else
                pub_img.publish(feature_points);

            // show track
            if (SHOW_TRACK)
            {
                cv::Mat imgTrack = trackerData.getTrackImage();
                pubTrackImage(imgTrack,time);
            }
        }
        ROS_INFO("whole feature tracker processing costs: %f", t_r.toc());
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "feature_tracker");
    ros::NodeHandle n("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
    readParameters(n);
    ROS_INFO("this is main round of feature tracker");

    trackerData.readIntrinsicParameter(CAM_NAMES);

    if(FISHEYE)
    {
        trackerData.fisheye_mask = cv::imread(FISHEYE_MASK, 0);
        if(!trackerData.fisheye_mask.data)
        {
            ROS_INFO("load mask fail");
            ROS_BREAK();
        }
        else
            ROS_INFO("load mask success");
    }

    ros::Subscriber sub_img = n.subscribe(IMAGE0_TOPIC, 100, img_callback);
    ros::Subscriber sub1_img = n.subscribe(IMAGE1_TOPIC, 100, img1_callback);

    pub_img = n.advertise<sensor_msgs::PointCloud>("feature", 1000);
    pub_match = n.advertise<sensor_msgs::Image>("feature_img",1000);
    pub_restart = n.advertise<std_msgs::Bool>("restart",1000);

    std::thread measurement_process{f_process};
    ros::spin();
    return 0;
}
