# Copyright 2021, Pick-It NV.
# All rights reserved.

## Contains all parameters describing reliability of detected object
## Values expressed as confidence levels ranging from 0.0 to 1.0

## Ratio of inlier points close to the border of the "Intermodalics hole" and
## all points of the "Intermodalics hole".
ReliabilityValue scene_coverage_2d

## Ratio of inlier points to all points in the cluster.
ReliabilityValue scene_coverage_3d

### TODO(@wannesvanloock): Add description pls.
ReliabilityValue model_contour_coverage_2d
### TODO(@wannesvanloock): Add description pls.
ReliabilityValue model_surface_coverage_2d

## 3D reliability of the matched model
ReliabilityValue model_coverage_3d

ReliabilityValue[] submodel_coverage_3d

ReliabilityValue matching_score_xd
