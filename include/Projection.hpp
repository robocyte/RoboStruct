#pragma once

#include "EigenTools.hpp"

// Solve for a 3x4 projection matrix, given a set of 3D points and 2D projections, optionally optimize the result (non-linear)
Mat34 ComputeProjectionMatrix(const Vec3Vec &points, const Vec2Vec &projections, bool optimize = false);

// Solve for a 3x4 projection matrix using RANSAC, given a set of 3D points and 2D projections
int ComputeProjectionMatrixRansac(const Vec3Vec &points, const Vec2Vec &projections,
								  int ransac_rounds, double ransac_threshold,
								  Mat34 *P);

void DecomposeProjectionMatrix(const Mat &P, Mat3 *K, Mat3 *R, Vec3 *t);
