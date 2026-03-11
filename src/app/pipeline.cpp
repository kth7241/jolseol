// src/app/pipeline.cpp
#include "app/pipeline.hpp"

#include <stdexcept>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "common/logging.hpp"
#include "common/clock.hpp"
#include "common/types.hpp"

#include "capture/camera.hpp"
#include "capture/gstreamer_camera.hpp"
#include "detect/detector.hpp"
#include "track/tracker.hpp"
#include "ipm/ipm_config.hpp"
#include "ipm/ipm_warper.hpp"
#include "zone/zone_manager.hpp"
#include "fusion/footpoint_mapper.hpp"
#include "speed/speed_estimator.hpp"
#include "ui/overlay.hpp"

namespace traffic::app {

struct Pipeline::Impl {
  // modules
  std::unique_ptr<traffic::ICamera> camera;
  std::unique_ptr<traffic::detect::IDetector> detector;
  std::unique_ptr<traffic::ITracker> tracker;

  traffic::IpmConfig ipm_cfg;
  std::unique_ptr<traffic::IpmWarper> ipm_warper;

  traffic::ZoneManager zone_mgr;

  traffic::FootpointMapper foot_mapper;
  traffic::SpeedEstimator speed_estimator;
  traffic::ui::Overlay overlay;

  explicit Impl(const Pipeline::Options& opt)
    : speed_estimator([&](){ traffic::SpeedEstimator::Options o; o.ema_alpha=opt.ema_alpha; o.dt_min=opt.dt_min; o.dt_max=opt.dt_max; o.max_speed_kmh=opt.max_speed_kmh; o.purge_timeout_sec=opt.purge_timeout_sec; return traffic::SpeedEstimator(o);}())
    , overlay(traffic::ui::Overlay::Options{}) {
    (void)opt;
  }
};

Pipeline::Pipeline(const Options& opt) : opt_(opt), impl_(std::make_unique<Impl>(opt)) {}
Pipeline::~Pipeline() = default;

static std::string defaultGstPipeline() {
  // 기본값: Jetson 환경은 프로젝트별로 다르므로, 일단 사용자가 채우는 것을 권장.
  // 이 문자열이 동작하지 않으면 Options.cam_pipeline에 자신의 pipeline을 넣어야 함.
  return "videotestsrc ! videoconvert ! video/x-raw,format=BGR ! appsink";
}

int Pipeline::run() {
  // 1) Camera
  traffic::CameraConfig cam_cfg;
  cam_cfg.name = "cam01";
  cam_cfg.source = "csi";
  cam_cfg.width = 1920;
  cam_cfg.height = 1080;
  cam_cfg.fps = 30;
  cam_cfg.pipeline = opt_.cam_pipeline.empty() ? defaultGstPipeline() : opt_.cam_pipeline;

  impl_->camera = std::make_unique<traffic::GStreamerCamera>();
  std::string cam_err;
  if (!impl_->camera->open(cam_cfg, &cam_err)) {
    LOGE("[Pipeline] camera open failed: %s", cam_err.c_str());
    return 1;
  }

  // 2) Detector (MVP: Stub)
  impl_->detector = traffic::detect::CreateDetectorStub();
  if (!impl_->detector) {
    LOGE("[Pipeline] detector create failed");
    return 2;
  }

  // 3) Tracker (IOU)
  impl_->tracker = traffic::CreateIouTracker(opt_.iou_threshold, opt_.max_age);
  if (!impl_->tracker) {
    LOGE("[Pipeline] tracker create failed");
    return 3;
  }

  // 4) IPM config/warper
  if (!opt_.ipm_yaml_path.empty()) {
    if (!traffic::load_ipm_config(opt_.ipm_yaml_path, impl_->ipm_cfg)) {
      LOGW("[Pipeline] IPM config load failed. Using default identity H/mpp.");
    }
  } else {
    LOGW("[Pipeline] ipm_yaml_path empty. Using default IPM config.");
  }
  impl_->ipm_warper = std::make_unique<traffic::IpmWarper>(impl_->ipm_cfg);

  // 5) Zone load (optional)
  if (!opt_.zones_json_path.empty()) {
    if (!impl_->zone_mgr.load(opt_.zones_json_path)) {
      LOGW("[Pipeline] zone load failed. Zone gating may not work.");
    }
  }

  cv::namedWindow(opt_.window_name, cv::WINDOW_NORMAL);

  traffic::FramePacket fp;
  std::vector<traffic::Detection> dets;
  std::vector<traffic::Track> tracks;
  std::vector<traffic::TrackFoot> feet;
  std::vector<traffic::SpeedResult> speeds;

  while (true) {
    fp = traffic::FramePacket{};
    if (!impl_->camera->grab(fp, nullptr)) {
      LOGW("[Pipeline] grab failed. continue...");
      continue;
    }
    if (fp.bgr.empty()) continue;

    // Detector uses full frame for now
    dets = impl_->detector->infer(fp.bgr);

    // Tracker
    tracks = impl_->tracker->update(dets, fp.ts_ns);

    // Footpoint mapping with zone fill (optional overload)
    feet = impl_->foot_mapper.map(tracks, *impl_->ipm_warper, impl_->zone_mgr);

    // Speed estimation
    speeds = impl_->speed_estimator.compute(feet, impl_->ipm_cfg.meters_per_pixel);

    // Draw overlay
    cv::Mat vis = fp.bgr; // shallow copy
    impl_->overlay.drawAll(vis, tracks, feet, speeds);

    // FPS info
    cv::putText(vis, "Press q to quit", cv::Point(10, vis.rows - 10),
                cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 2, cv::LINE_AA);

    cv::imshow(opt_.window_name, vis);
    const int key = cv::waitKey(1);
    if (key == 'q' || key == 'Q' || key == 27) break; // q or ESC
  }

  impl_->camera->close();
  cv::destroyWindow(opt_.window_name);
  return 0;
}

} // namespace traffic::app
