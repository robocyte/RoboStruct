#pragma once

#include "EigenTools.hpp"

int ComputeRelativePoseRansac(  const Point2Vec &pts1, const Point2Vec &pts2,
                                const Mat &K1, const Mat &K2,
                                double ransac_threshold, int ransac_rounds,
                                Mat3 *R, Vec3 *t);
