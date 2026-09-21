#include "convex_plane_decomposition/PlaneDecompositionPipeline.h"

#include <opencv2/core/eigen.hpp>

#include <cmath>
#include <stdexcept>

#include <grid_map_core/iterators/GridMapIterator.hpp>

namespace convex_plane_decomposition {

PlaneDecompositionPipeline::PlaneDecompositionPipeline(const Config& config)
    : preprocessing_(config.preprocessingParameters),
      slidingWindowPlaneExtractor_(config.slidingWindowPlaneExtractorParameters, config.ransacPlaneExtractorParameters),
      contourExtraction_(config.contourExtractionParameters),
      postprocessing_(config.postprocessingParameters) {}

void PlaneDecompositionPipeline::update(grid_map::GridMap&& gridMap, const std::string& elevationLayer) {
  if (!gridMap.exists(elevationLayer)) {
    throw std::invalid_argument("PlaneDecompositionPipeline: elevation layer is missing");
  }
  if (gridMap.getSize().x() <= 0 || gridMap.getSize().y() <= 0 || !containsFiniteValue(gridMap.get(elevationLayer))) {
    throw std::invalid_argument("PlaneDecompositionPipeline: elevation map is empty or has no finite values");
  }

  const grid_map::GridMap observedInput = gridMap;

  // Clear / Overwrite old result
  planarTerrain_.planarRegions.clear();
  planarTerrain_.gridMap = std::move(gridMap);

  preprocessTimer_.startTimer();
  preprocessing_.preprocess(planarTerrain_.gridMap, elevationLayer);
  preprocessTimer_.endTimer();

  grid_map::Matrix observedMask = grid_map::Matrix::Zero(planarTerrain_.gridMap.getSize()(0), planarTerrain_.gridMap.getSize()(1));
  for (grid_map::GridMapIterator iterator(planarTerrain_.gridMap); !iterator.isPastEnd(); ++iterator) {
    grid_map::Position position;
    grid_map::Index observedIndex;
    if (planarTerrain_.gridMap.getPosition(*iterator, position) && observedInput.getIndex(position, observedIndex) &&
        std::isfinite(observedInput.at(elevationLayer, observedIndex))) {
      observedMask((*iterator)(0), (*iterator)(1)) = 1.0F;
    }
  }

  slidingWindowTimer_.startTimer();
  slidingWindowPlaneExtractor_.runExtraction(planarTerrain_.gridMap, elevationLayer, &observedMask);
  slidingWindowTimer_.endTimer();

  contourExtractionTimer_.startTimer();
  planarTerrain_.planarRegions = contourExtraction_.extractPlanarRegions(slidingWindowPlaneExtractor_.getSegmentedPlanesMap());
  contourExtractionTimer_.endTimer();

  postprocessTimer_.startTimer();
  // Add binary map
  const std::string planeClassificationLayer{"plane_classification"};
  planarTerrain_.gridMap.add(planeClassificationLayer);
  auto& traversabilityMask = planarTerrain_.gridMap.get(planeClassificationLayer);
  cv::cv2eigen(slidingWindowPlaneExtractor_.getBinaryLabeledImage(), traversabilityMask);

  postprocessing_.postprocess(planarTerrain_, elevationLayer, planeClassificationLayer);
  postprocessTimer_.endTimer();
}

void PlaneDecompositionPipeline::getSegmentation(grid_map::GridMap::Matrix& segmentation) const {
  cv::cv2eigen(slidingWindowPlaneExtractor_.getSegmentedPlanesMap().labeledImage, segmentation);
}

}  // namespace convex_plane_decomposition
