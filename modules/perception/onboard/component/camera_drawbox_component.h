#pragma once

#include <map>
#include <memory>
#include <string>
#include <vector>
#include "cyber/base/concurrent_object_pool.h"

#include "cyber/component/component.h"
#include "modules/drivers/proto/sensor_image.pb.h"
#include "modules/perception/base/object.h"
#include "modules/perception/base/object_types.h"
#include "modules/perception/base/point.h"
#include "modules/perception/camera/app/cipv_camera.h"
#include "modules/perception/camera/app/obstacle_camera_perception.h"
#include "modules/perception/camera/app/perception.pb.h"
#include "modules/perception/camera/common/util.h"
#include "modules/perception/camera/lib/interface/base_camera_perception.h"
#include "modules/perception/camera/tools/offline/visualizer.h"
#include "modules/perception/onboard/component/camera_perception_viz_message.h"
#include "modules/perception/onboard/inner_component_messages/inner_component_messages.h"
#include "modules/perception/onboard/proto/fusion_camera_detection_component.pb.h"
#include "modules/perception/onboard/transform_wrapper/transform_wrapper.h"
#include "modules/perception/proto/motion_service.pb.h"
#include "modules/perception/proto/perception_camera.pb.h"
#include "modules/perception/proto/perception_obstacle.pb.h"
namespace apollo {
namespace perception {
namespace onboard {

using apollo::cyber::base::CCObjectPool;


class CameraDrawboxComponent : public apollo::cyber::Component<> {
 public:
  CameraDrawboxComponent()=default;
  ~CameraDrawboxComponent()=default;

  CameraDrawboxComponent(const CameraDrawboxComponent&) =
      delete;
  CameraDrawboxComponent& operator=(
      const CameraDrawboxComponent&) = delete;
 bool Init() override;

private:
      //message receiving
      void OnReceiveImage(const std::shared_ptr<apollo::drivers::Image>& in_message);
      void OnReceiveObstacle(const std::shared_ptr<apollo::perception::camera::CameraDebug> &debugmessage);

      std::shared_ptr<apollo::drivers::CompressedImage> output_image;
      std::shared_ptr<CCObjectPool<apollo::drivers::CompressedImage>> image_pool_ = nullptr;

      // draw rectangle
      void DrawRect(const std::shared_ptr<apollo::perception::camera::CameraDebug> &debugmessage);

      //Readers
      std::shared_ptr<
            apollo::cyber::Reader<apollo::perception::camera::CameraDebug>>
            camera_debug_reader_;
      std::shared_ptr<
            apollo::cyber::Reader<apollo::drivers::Image>>
            image_reader_;
      std::shared_ptr<
            apollo::cyber::Writer<apollo::drivers::CompressedImage>>
            box_writer_;
      
      // image 
      cv::Mat image;
      int width=1920;
      int height=1080;

      // colors of box
      std::vector<cv::Scalar> colors;

      //flag for receiving image
      bool image_received;
};

CYBER_REGISTER_COMPONENT(CameraDrawboxComponent);

}
}
}