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
    std::string camerachannel="/perception/camera_debug";
    std::string imagechannel="/apollo/sensor/camera/front_12mm/image";

    camera_debug_reader_ = node_->CreateReader<apollo::perception::camera::CameraDebug>(camerachannel,
                       [this]( const std::shared_ptr<apollo::perception::camera::CameraDebug> &debugmessage)
                       {OnReceiveObstacle(debugmessage);});
    image_reader_ = node_->CreateReader<apollo::drivers::Image>(imagechannel,
                       [this]( const std::shared_ptr<apollo::drivers::Image>& in_message)
                       { OnReceiveImage(in_message);});
    
    //initialize
    cv::Mat image(width,height,CV_8UC3);
    image_received = false;
    return true;
}


void CameraDrawboxComponent::OnReceiveObstacle(const std::shared_ptr<apollo::perception::camera::CameraDebug> &debugmessage)
{
    if (image_received)
    {
        CameraDrawboxComponent::DrawRect(debugmessage)
    }
}


void CameraDrawboxComponent::OnReceiveImage(const std::shared_ptr<apollo::drivers::Image>& in_message)
{
    cv::Mat image(in_message->height(),in_message->width(),CV_8UC3,const_cast<char*>(in_message->data().data()),in_message->step());
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
    cv::rectangle(image, p0, p1, cv::Scalar(0, 0, 255), 50 , 8);
    }

    if(!image.empty()){
        cv::imwrite("zwrrwz.png",image);
        //TODO: imshow() does not work in apollo
        cv::imshow("image",image);
    }
}

} //onboard
} //perception
} //apollo
