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
    camera_debug_reader_=node_->CreateReader<apollo::perception::camera::CameraDebug>(camerachannel,
                       [this]( const std::shared_ptr<apollo::perception::camera::CameraDebug> &debugmessage)
                       {drawrect(debugmessage);});
    top_left->set_x(0);
    top_left->set_x(0);
    low_right->set_x(0);
    low_right->set_x(0);
    cv::Mat image(width,height,CV_8UC3,cv::Scalar(0,0,255));
    image_reader_=node_->CreateReader<apollo::drivers::Image>(imagechannel,
                       [this]( const std::shared_ptr<apollo::drivers::Image>& in_message)
                       { OnReceiveImage(in_message);});
    return true;
}



void CameraDrawboxComponent::drawrect(const std::shared_ptr<apollo::perception::camera::CameraDebug> &debugmessage)
{
    cv::Point p0;
    cv::Point p1;

    for(repeatedobs::iterator it=debugmessage->camera_obstacle().begin();it!=debugmessage->camera_obstacle().end();it++)
    {
        // top_left=&(it->upper_left());
        // low_right=&(it->lower_right());
        
    p0= cv::Point(it->upper_left().x(),it->upper_left().y());
    p1= cv::Point(it->lower_right().x(),it->lower_right().y());

    cv::rectangle(image, p0, p1, cv::Scalar(0, 0, 255), 50 , 8);

    }
     cv::imshow("image",image);
}

void CameraDrawboxComponent::OnReceiveImage(const std::shared_ptr<apollo::drivers::Image>& in_message)
{
    image.data=( uint8_t *)(in_message->data().data());
}

}
}
}
