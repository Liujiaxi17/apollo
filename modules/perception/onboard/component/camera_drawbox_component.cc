#include "modules/perception/onboard/component/camera_drawbox_component.h"

#include "absl/strings/str_cat.h"
#include "boost/algorithm/string.hpp"
#include "boost/format.hpp"
#include "yaml-cpp/yaml.h"

#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "modules/common/math/math_utils.h"
#include "modules/common/time/time.h"
#include "modules/common/time/time_util.h"
#include "modules/perception/common/perception_gflags.h"
#include "modules/perception/common/sensor_manager/sensor_manager.h"
#include "modules/perception/onboard/common_flags/common_flags.h"
#include "modules/perception/onboard/component/camera_perception_viz_message.h"

namespace apollo {
namespace perception {
namespace onboard {

using apollo::cyber::common::GetAbsolutePath;
typedef  google::protobuf::RepeatedPtrField<const apollo::perception::camera::CameraObstacle> repeatedobs;


bool CameraDrawboxComponent::Init()
{
    //TODO: you'd better specify the channel names in proto file 
    //but it doesnt matter if you dont do it.
    std::string camerachannel="/perception/camera_debug";
    std::string boxed_image_channel = "/perception/camera_boxed";
    //6mm camera provides a wider area
    std::string imagechannel="/apollo/sensor/camera/front_6mm/image";

    camera_debug_reader_ = node_->CreateReader<apollo::perception::camera::CameraDebug>(camerachannel,
                       [this]( const std::shared_ptr<apollo::perception::camera::CameraDebug> &debugmessage)
                       {OnReceiveObstacle(debugmessage);});
    image_reader_ = node_->CreateReader<apollo::drivers::Image>(imagechannel,
                       [this]( const std::shared_ptr<apollo::drivers::Image>& in_message)
                       { OnReceiveImage(in_message);});

    //a bug will occur in function OnReceiveImage if using msg type Image.
    //So I change the msg type to Compressed Image
    box_writer_ = node_->CreateWriter<apollo::drivers::CompressedImage>(boxed_image_channel);

    // referring to other files using compressed image
    image_pool_.reset(new CCObjectPool<apollo::drivers::CompressedImage>(100));
    image_pool_->ConstructAll();

    //initialize
    cv::Mat image(width,height,CV_8UC3);
    image_received = false;
    
    // colors init, more beautiful colors required
    // UNKNOWN = 0;
    // UNKNOWN_MOVABLE = 1;
    // UNKNOWN_UNMOVABLE = 2;
    // PEDESTRIAN = 3;  // Pedestrian, usually determined by moving behaviour.
    // BICYCLE = 4;     // bike, motor bike
    // VEHICLE = 5;  
    colors.push_back(cv::Scalar(0, 0, 255)); 
    colors.push_back(cv::Scalar(0, 200, 200)); 
    colors.push_back(cv::Scalar(200, 0, 255)); 
    colors.push_back(cv::Scalar(0, 100, 100)); 
    colors.push_back(cv::Scalar(155, 155, 255)); 
    colors.push_back(cv::Scalar(0, 0, 0)); 
    
    std::cout<<"init"<<std::endl;
    return true;
}


void CameraDrawboxComponent::OnReceiveObstacle(const std::shared_ptr<apollo::perception::camera::CameraDebug> &debugmessage)
{
    // Here, camera_debug msg comes from two camera, 6mm and 12mm. Choose the first one
    if (image_received && debugmessage->source_topic()[28] == '6')
    {
        CameraDrawboxComponent::DrawRect(debugmessage);
    }
}


void CameraDrawboxComponent::OnReceiveImage(const std::shared_ptr<apollo::drivers::Image>& in_message)
{
    // Here if simply using:
    // cv::Mat image(in_message->height(),in_message->width(),CV_8UC3,const_cast<char*>(in_message->data().data()),in_message->step());
    // it seems to create a temprary variable instead of changing the image variable defined in head file.
    cv::Mat tempimage(in_message->height(),in_message->width(),CV_8UC3,const_cast<char*>(in_message->data().data()),in_message->step());
    image = tempimage;

    //copied from similar file. Defining other info of CompressImage
    output_image = image_pool_->GetObject();
    output_image->set_measurement_time(in_message->measurement_time());
    output_image->mutable_header()->CopyFrom(in_message->header());
    output_image->set_frame_id(in_message->frame_id());
    output_image->set_format(in_message->encoding() + "; jpeg compressed bgr8");

    image_received = true;
}

void CameraDrawboxComponent::DrawRect(const std::shared_ptr<apollo::perception::camera::CameraDebug> &debugmessage)
{
    // corner points of rectangle
    cv::Point p0;
    cv::Point p1;

    for(repeatedobs::iterator it=debugmessage->camera_obstacle().begin();it!=debugmessage->camera_obstacle().end();it++)
    {
    // for each object, draw a rectangle
    p0= cv::Point(it->upper_left().x(),it->upper_left().y());
    p1= cv::Point(it->lower_right().x(),it->lower_right().y());
    cv::rectangle(image, p0, p1, colors[it->type()], 20 , 8);
    }

    // specify output data
    std::vector<uint8_t> temp_buffer;
    cv::imencode(".jpg", image, temp_buffer);
    output_image->set_data(temp_buffer.data(),temp_buffer.size());
    
    box_writer_->Write(output_image);
}

} //onboard
} //perception
} //apollo
