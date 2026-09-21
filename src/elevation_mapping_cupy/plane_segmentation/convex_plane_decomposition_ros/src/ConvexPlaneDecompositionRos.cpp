#include "convex_plane_decomposition_ros/ConvexPlaneDecompositionRos.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <utility>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <grid_map_core/GridMap.hpp>
#include <grid_map_core/iterators/GridMapIterator.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <tf2/time.h>
#include <tf2_eigen/tf2_eigen.hpp>

#include "convex_plane_decomposition_ros/MessageConversion.h"
#include "convex_plane_decomposition_ros/ParameterLoading.h"
#include "convex_plane_decomposition_ros/RosVisualizations.h"

namespace convex_plane_decomposition {
namespace {

constexpr int64_t kThrottleMilliseconds = 5000;
constexpr size_t kTimingWindowSize = 1024;

bool finitePose(const geometry_msgs::msg::Pose& pose) {
  return std::isfinite(pose.position.x) && std::isfinite(pose.position.y) && std::isfinite(pose.position.z) &&
         std::isfinite(pose.orientation.x) && std::isfinite(pose.orientation.y) && std::isfinite(pose.orientation.z) &&
         std::isfinite(pose.orientation.w);
}

diagnostic_msgs::msg::KeyValue keyValue(const std::string& key, const std::string& value) {
  diagnostic_msgs::msg::KeyValue item;
  item.key = key;
  item.value = value;
  return item;
}

template <typename T>
std::string asString(const T& value) {
  std::ostringstream stream;
  stream << value;
  return stream.str();
}

double polygonArea(const CgalPolygonWithHoles2d& polygon) {
  double area = std::abs(polygon.outer_boundary().area());
  for (const auto& hole : polygon.holes()) {
    area -= std::abs(hole.area());
  }
  return std::max(0.0, area);
}

}  // namespace

ConvexPlaneExtractionROS::ConvexPlaneExtractionROS(const rclcpp::NodeOptions& options)
    : Node("plane_segmentation_node", options), tfBuffer_(get_clock()), tfListener_(tfBuffer_, this, true) {
  elevationMapTopic_ = declare_parameter<std::string>("input_topic", "/elevation_mapping_node/elevation_map_raw");
  elevationLayer_ = declare_parameter<std::string>("elevation_layer", "elevation");
  varianceLayer_ = declare_parameter<std::string>("variance_layer", "variance");
  targetFrameId_ = declare_parameter<std::string>("target_frame", "base_link");
  roiCenterFrameId_ = declare_parameter<std::string>("roi.center_frame", "");
  processingRateHz_ = declare_parameter<double>("processing_rate_hz", 5.0);
  inputTimeoutSec_ = declare_parameter<double>("input_timeout_sec", 1.0);
  tfTimeoutSec_ = declare_parameter<double>("tf_timeout_sec", 0.1);
  roiSideLength_ = declare_parameter<double>("roi.side_length", 2.0);
  minimumValidCells_ = declare_parameter<int64_t>("minimum_valid_cells", 25);
  useVarianceLayer_ = declare_parameter<bool>("validity.use_variance", false);
  maximumVariance_ = declare_parameter<double>("validity.maximum_variance", 0.05);
  minimumRegionArea_ = declare_parameter<double>("minimum_region_area", 0.04);
  publishPlanarTerrain_ = declare_parameter<bool>("publish_planar_terrain", true);
  publishDebugOutputs_ = declare_parameter<bool>("publish_debug_outputs", true);
  markerLineWidth_ = declare_parameter<double>("marker_line_width", 0.005);

  if (elevationMapTopic_.empty() || elevationLayer_.empty() || targetFrameId_.empty() || roiCenterFrameId_.empty()) {
    throw std::invalid_argument("input_topic, elevation_layer, target_frame, and roi.center_frame must be non-empty");
  }
  if (!(processingRateHz_ > 0.0) || !(inputTimeoutSec_ > 0.0) || tfTimeoutSec_ < 0.0 || !(roiSideLength_ > 0.0) ||
      minimumValidCells_ < 3 || minimumRegionArea_ < 0.0 || markerLineWidth_ <= 0.0) {
    throw std::invalid_argument("invalid processing, timeout, ROI, valid-cell, area, or marker parameter");
  }
  if (useVarianceLayer_ && (varianceLayer_.empty() || maximumVariance_ < 0.0)) {
    throw std::invalid_argument("variance validation requires a layer name and a non-negative maximum_variance");
  }

  PlaneDecompositionPipeline::Config config;
  config.preprocessingParameters = loadPreprocessingParameters(this, "preprocessing");
  config.contourExtractionParameters = loadContourExtractionParameters(this, "contour_extraction");
  config.ransacPlaneExtractorParameters = loadRansacPlaneExtractorParameters(this, "ransac_plane_refinement");
  config.slidingWindowPlaneExtractorParameters =
      loadSlidingWindowPlaneExtractorParameters(this, "sliding_window_plane_extractor");
  config.postprocessingParameters = loadPostprocessingParameters(this, "postprocessing");
  planeDecompositionPipeline_ = std::make_unique<PlaneDecompositionPipeline>(config);

  const auto depthOne = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile();
  elevationMapSubscriber_ = create_subscription<GridMapMessage>(
      elevationMapTopic_, depthOne, std::bind(&ConvexPlaneExtractionROS::mapCallback, this, std::placeholders::_1));
  regionPublisher_ = create_publisher<convex_plane_decomposition_msgs::msg::PlanarTerrain>("~/planar_terrain", depthOne);
  segmentationMapPublisher_ = create_publisher<GridMapMessage>("~/segmentation_map", depthOne);
  boundaryPublisher_ = create_publisher<visualization_msgs::msg::MarkerArray>("~/boundaries", depthOne);
  insetPublisher_ = create_publisher<visualization_msgs::msg::MarkerArray>("~/insets", depthOne);
  diagnosticsPublisher_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("~/diagnostics", depthOne);

  processingTimer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / processingRateHz_), std::bind(&ConvexPlaneExtractionROS::processLatestMap, this));
  diagnosticsTimer_ = create_wall_timer(std::chrono::seconds(1), std::bind(&ConvexPlaneExtractionROS::publishDiagnostics, this));
  parameterCallbackHandle_ = add_on_set_parameters_callback(
      std::bind(&ConvexPlaneExtractionROS::onSetParameters, this, std::placeholders::_1));

  RCLCPP_INFO(get_logger(),
              "Listening to %s with KeepLast(1); processing at %.3f Hz in frame '%s'; %.3f m square ROI centered at '%s'",
              elevationMapTopic_.c_str(), processingRateHz_, targetFrameId_.c_str(), roiSideLength_, roiCenterFrameId_.c_str());
}

rcl_interfaces::msg::SetParametersResult ConvexPlaneExtractionROS::onSetParameters(
    const std::vector<rclcpp::Parameter>& parameters) {
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  std::lock_guard<std::mutex> lock(roiConfigMutex_);
  std::string centerFrame = roiCenterFrameId_;
  double sideLength = roiSideLength_;
  for (const auto& parameter : parameters) {
    if (parameter.get_name() == "roi.center_frame") {
      centerFrame = parameter.as_string();
      if (centerFrame.empty()) {
        result.successful = false;
        result.reason = "roi.center_frame must be non-empty";
        return result;
      }
    } else if (parameter.get_name() == "roi.side_length") {
      sideLength = parameter.as_double();
      if (!std::isfinite(sideLength) || sideLength <= 0.0) {
        result.successful = false;
        result.reason = "roi.side_length must be finite and positive";
        return result;
      }
    }
  }
  roiCenterFrameId_ = std::move(centerFrame);
  roiSideLength_ = sideLength;
  return result;
}

void ConvexPlaneExtractionROS::mapCallback(GridMapMessage::ConstSharedPtr message) {
  std::lock_guard<std::mutex> lock(latestMapMutex_);
  if (latestMapPending_) {
    ++overwrittenMaps_;
  }
  latestMap_ = std::move(message);
  latestMapPending_ = true;
  latestReceiveTime_ = now();
  ++receivedMaps_;
  if (receivedMaps_ == 1) {
    firstReceiveSteady_ = std::chrono::steady_clock::now();
  }
}

bool ConvexPlaneExtractionROS::validateMessage(const GridMapMessage& message, std::string& reason) const {
  if (message.header.frame_id.empty()) {
    reason = "input frame_id is empty";
    return false;
  }
  if (!std::isfinite(message.info.resolution) || message.info.resolution <= 0.0 || !std::isfinite(message.info.length_x) ||
      !std::isfinite(message.info.length_y) || message.info.length_x <= 0.0 || message.info.length_y <= 0.0 ||
      !finitePose(message.info.pose)) {
    reason = "GridMap geometry contains empty or non-finite values";
    return false;
  }
  if (message.layers.size() != message.data.size()) {
    reason = "GridMap layers and data arrays have different sizes";
    return false;
  }
  if (std::find(message.layers.begin(), message.layers.end(), elevationLayer_) == message.layers.end()) {
    reason = "required elevation layer '" + elevationLayer_ + "' is missing";
    return false;
  }
  if (useVarianceLayer_ && std::find(message.layers.begin(), message.layers.end(), varianceLayer_) == message.layers.end()) {
    reason = "configured variance layer '" + varianceLayer_ + "' is missing";
    return false;
  }
  return true;
}

bool ConvexPlaneExtractionROS::prepareRoi(const GridMapMessage& message, grid_map::GridMap& roi, std::string& reason) {
  std::vector<std::string> layers{elevationLayer_};
  if (useVarianceLayer_) {
    layers.push_back(varianceLayer_);
  }

  grid_map::GridMap map;
  if (!grid_map::GridMapRosConverter::fromMessage(message, map, layers, false, false) || !map.exists(elevationLayer_)) {
    reason = "GridMap conversion failed";
    return false;
  }
  if (map.getSize().x() <= 0 || map.getSize().y() <= 0) {
    reason = "GridMap is empty";
    return false;
  }

  if (message.header.frame_id != targetFrameId_) {
    try {
      const rclcpp::Time stamp(message.header.stamp, get_clock()->get_clock_type());
      const auto transform = tfBuffer_.lookupTransform(targetFrameId_, message.header.frame_id, stamp,
                                                       tf2::durationFromSec(tfTimeoutSec_));
      map = map.getTransformedMap(tf2::transformToEigen(transform), elevationLayer_, targetFrameId_);
      map.setTimestamp(stamp.nanoseconds());
    } catch (const tf2::TransformException& exception) {
      reason = std::string("TF unavailable at input timestamp: ") + exception.what();
      return false;
    }
  }

  std::string centerFrame;
  double sideLength = 0.0;
  {
    std::lock_guard<std::mutex> lock(roiConfigMutex_);
    centerFrame = roiCenterFrameId_;
    sideLength = roiSideLength_;
  }
  const rclcpp::Time stamp(message.header.stamp, get_clock()->get_clock_type());
  grid_map::Position roiCenter = grid_map::Position::Zero();
  if (centerFrame != targetFrameId_) {
    try {
      const auto centerTransform =
          tfBuffer_.lookupTransform(targetFrameId_, centerFrame, stamp, tf2::durationFromSec(tfTimeoutSec_));
      roiCenter.x() = centerTransform.transform.translation.x;
      roiCenter.y() = centerTransform.transform.translation.y;
    } catch (const tf2::TransformException& exception) {
      reason = std::string("ROI center TF unavailable at input timestamp: ") + exception.what();
      return false;
    }
  }
  grid_map::Index roiCenterIndex;
  if (!map.getIndex(roiCenter, roiCenterIndex) || !map.getPosition(roiCenterIndex, roiCenter)) {
    reason = "ROI center lies outside the input map";
    return false;
  }
  bool success = false;
  roi = map.getSubmap(roiCenter, grid_map::Length(sideLength, sideLength), success);
  if (!success || roi.getSize().x() <= 0 || roi.getSize().y() <= 0) {
    reason = "requested ROI does not overlap the transformed map";
    return false;
  }
  roi.setFrameId(targetFrameId_);
  roi.setTimestamp(rclcpp::Time(message.header.stamp, get_clock()->get_clock_type()).nanoseconds());

  auto& elevation = roi.get(elevationLayer_);
  const grid_map::Matrix* variance = useVarianceLayer_ ? &roi.get(varianceLayer_) : nullptr;
  int64_t validCells = 0;
  for (Eigen::Index row = 0; row < elevation.rows(); ++row) {
    for (Eigen::Index col = 0; col < elevation.cols(); ++col) {
      const bool validElevation = std::isfinite(elevation(row, col));
      const bool validVariance = variance == nullptr ||
                                 (std::isfinite((*variance)(row, col)) && (*variance)(row, col) <= maximumVariance_);
      if (validElevation && validVariance) {
        ++validCells;
      } else {
        elevation(row, col) = std::numeric_limits<float>::quiet_NaN();
      }
    }
  }
  if (validCells < minimumValidCells_) {
    reason = "ROI has only " + std::to_string(validCells) + " valid cells; minimum is " + std::to_string(minimumValidCells_);
    return false;
  }

  lastRows_ = static_cast<size_t>(elevation.rows());
  lastCols_ = static_cast<size_t>(elevation.cols());
  return true;
}

void ConvexPlaneExtractionROS::filterSmallRegions(PlanarTerrain& terrain) const {
  terrain.planarRegions.erase(
      std::remove_if(terrain.planarRegions.begin(), terrain.planarRegions.end(), [this](const PlanarRegion& region) {
        return polygonArea(region.boundaryWithInset.boundary) < minimumRegionArea_;
      }),
      terrain.planarRegions.end());
}

void ConvexPlaneExtractionROS::recordProcessingTime(double milliseconds) {
  if (processingTimesMs_.size() == kTimingWindowSize) {
    processingTimesMs_.erase(processingTimesMs_.begin());
  }
  processingTimesMs_.push_back(milliseconds);
}

void ConvexPlaneExtractionROS::processLatestMap() {
  GridMapMessage::ConstSharedPtr message;
  rclcpp::Time receiveTime(0, 0, get_clock()->get_clock_type());
  {
    std::lock_guard<std::mutex> lock(latestMapMutex_);
    if (!latestMapPending_ || latestMap_ == nullptr) {
      if (latestMap_ != nullptr && (now() - latestReceiveTime_).seconds() > inputTimeoutSec_) {
        lastStatus_ = "input timeout";
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), kThrottleMilliseconds, "No fresh GridMap received for %.3f s",
                             inputTimeoutSec_);
      }
      return;
    }
    message = latestMap_;
    receiveTime = latestReceiveTime_;
    latestMapPending_ = false;
  }

  if ((now() - receiveTime).seconds() > inputTimeoutSec_) {
    ++rejectedMaps_;
    lastStatus_ = "rejected stale input";
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), kThrottleMilliseconds, "Rejecting stale GridMap");
    return;
  }

  const auto start = std::chrono::steady_clock::now();
  std::string reason;
  if (!validateMessage(*message, reason)) {
    ++rejectedMaps_;
    lastStatus_ = reason;
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), kThrottleMilliseconds, "%s", reason.c_str());
    return;
  }

  try {
    grid_map::GridMap roi;
    if (!prepareRoi(*message, roi, reason)) {
      ++rejectedMaps_;
      lastStatus_ = reason;
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), kThrottleMilliseconds, "%s", reason.c_str());
      return;
    }

    const grid_map::GridMap rawRoi = roi;
    planeDecompositionPipeline_->update(std::move(roi), elevationLayer_);
    auto& terrain = planeDecompositionPipeline_->getPlanarTerrain();
    filterSmallRegions(terrain);

    auto terrainMessage = toMessage(terrain);
    rclcpp::Serialization<convex_plane_decomposition_msgs::msg::PlanarTerrain> serializer;
    rclcpp::SerializedMessage serialized;
    serializer.serialize_message(&terrainMessage, &serialized);
    lastOutputBytes_ = serialized.size();

    grid_map::Matrix rawElevation = grid_map::Matrix::Constant(
        terrain.gridMap.getSize()(0), terrain.gridMap.getSize()(1), std::numeric_limits<float>::quiet_NaN());
    for (grid_map::GridMapIterator iterator(terrain.gridMap); !iterator.isPastEnd(); ++iterator) {
      grid_map::Position position;
      grid_map::Index rawIndex;
      if (terrain.gridMap.getPosition(*iterator, position) && rawRoi.getIndex(position, rawIndex)) {
        rawElevation((*iterator)(0), (*iterator)(1)) = rawRoi.at(elevationLayer_, rawIndex);
      }
    }
    terrain.gridMap.add("elevation_raw", rawElevation);
    terrain.gridMap.add("segmentation_candidates");
    planeDecompositionPipeline_->getSegmentation(terrain.gridMap.get("segmentation_candidates"));
    terrain.gridMap.add("plane_visualization_height", terrain.gridMap.get(elevationLayer_ + "_before_postprocess"));
    auto& visualizationHeight = terrain.gridMap.get("plane_visualization_height");
    for (Eigen::Index row = 0; row < rawElevation.rows(); ++row) {
      for (Eigen::Index col = 0; col < rawElevation.cols(); ++col) {
        if (!std::isfinite(rawElevation(row, col))) {
          visualizationHeight(row, col) = std::numeric_limits<float>::quiet_NaN();
        }
      }
    }

    if (publishPlanarTerrain_) {
      regionPublisher_->publish(terrainMessage);
    }
    if (publishDebugOutputs_) {
      segmentationMapPublisher_->publish(*grid_map::GridMapRosConverter::toMessage(terrain.gridMap));
      boundaryPublisher_->publish(convertBoundariesToRosMarkers(terrain.planarRegions, terrain.gridMap.getFrameId(),
                                                                 terrain.gridMap.getTimestamp(), markerLineWidth_));
      insetPublisher_->publish(convertInsetsToRosMarkers(terrain.planarRegions, terrain.gridMap.getFrameId(),
                                                          terrain.gridMap.getTimestamp(), markerLineWidth_));
    }

    ++processedMaps_;
    if (processedMaps_ == 1) {
      firstProcessSteady_ = std::chrono::steady_clock::now();
    }
    lastStatus_ = "ok: " + std::to_string(terrain.planarRegions.size()) + " planar regions";
  } catch (const std::exception& exception) {
    ++rejectedMaps_;
    lastStatus_ = std::string("processing exception: ") + exception.what();
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), kThrottleMilliseconds, "%s", lastStatus_.c_str());
  }

  const auto elapsed = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
  recordProcessingTime(elapsed);
}

void ConvexPlaneExtractionROS::publishDiagnostics() {
  diagnostic_msgs::msg::DiagnosticArray array;
  array.header.stamp = now();
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = get_fully_qualified_name() + std::string(": plane segmentation");
  status.hardware_id = "none";
  status.level = lastStatus_.rfind("ok:", 0) == 0 ? diagnostic_msgs::msg::DiagnosticStatus::OK
                                                   : diagnostic_msgs::msg::DiagnosticStatus::WARN;
  status.message = lastStatus_;

  const auto nowSteady = std::chrono::steady_clock::now();
  const double receiveSeconds = receivedMaps_ > 1 ? std::chrono::duration<double>(nowSteady - firstReceiveSteady_).count() : 0.0;
  const double processSeconds = processedMaps_ > 1 ? std::chrono::duration<double>(nowSteady - firstProcessSteady_).count() : 0.0;
  status.values.push_back(keyValue("received_maps", asString(receivedMaps_)));
  status.values.push_back(keyValue("processed_maps", asString(processedMaps_)));
  status.values.push_back(keyValue("rejected_maps", asString(rejectedMaps_)));
  status.values.push_back(keyValue("overwritten_pending_maps", asString(overwrittenMaps_)));
  status.values.push_back(keyValue("input_hz", receiveSeconds > 0.0 ? asString((receivedMaps_ - 1) / receiveSeconds) : "n/a"));
  status.values.push_back(keyValue("processing_hz", processSeconds > 0.0 ? asString((processedMaps_ - 1) / processSeconds) : "n/a"));
  status.values.push_back(keyValue("last_grid_rows", asString(lastRows_)));
  status.values.push_back(keyValue("last_grid_cols", asString(lastCols_)));
  status.values.push_back(keyValue("last_planar_terrain_bytes", asString(lastOutputBytes_)));
  status.values.push_back(keyValue("gpu_used", "false"));

  if (!processingTimesMs_.empty()) {
    auto sorted = processingTimesMs_;
    std::sort(sorted.begin(), sorted.end());
    const auto percentile = [&sorted](double fraction) {
      const size_t index = static_cast<size_t>(std::floor(fraction * static_cast<double>(sorted.size() - 1)));
      return sorted[index];
    };
    status.values.push_back(keyValue("processing_ms_p50", asString(percentile(0.50))));
    status.values.push_back(keyValue("processing_ms_p95", asString(percentile(0.95))));
    status.values.push_back(keyValue("processing_ms_p99", asString(percentile(0.99))));
    status.values.push_back(keyValue("processing_ms_max", asString(sorted.back())));
  }

  array.status.push_back(std::move(status));
  diagnosticsPublisher_->publish(array);
}

}  // namespace convex_plane_decomposition
