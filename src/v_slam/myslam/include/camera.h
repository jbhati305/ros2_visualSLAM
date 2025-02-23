/*
 * Description: Declares the Camera class which encapsulates camera intrinsics and pose.
 * It provides methods for coordinate transformations between world, camera, and pixel spaces.
 *
 * Transform functions:
 * 1. world2camera:
 *    p_c = R * p_w + t, where T_c_w = [R|t]
 *
 * 2. camera2world:
 *    p_w = R⁻¹ * (p_c - t)
 *
 * 3. camera2pixel:
 *    Given a point p_c = [x, y, z]ᵀ,
 *    u = fx * (x/z) + cx,  v = fy * (y/z) + cy
 *
 * 4. pixel2camera:
 *    For a pixel [u, v] with depth d,
 *    p_c = [ (u - cx)/fx * d,  (v - cy)/fy * d,  d ]
 *
 * 5. pixel2world:
 *    p_w = camera2world(pixel2camera([u, v], d), T_c_w)
 *
 * 6. world2pixel:
 *    p_pixel = camera2pixel(world2camera(p_w, T_c_w))
 *
 * Intrinsic matrix K:
 *      [ fx   0   cx ]
 *      [  0  fy   cy ]
 *      [  0   0    1 ]
 */

#pragma once
#ifndef MYSLAM_CAMERA_H
#define MYSLAM_CAMERA_H

#include "common_include.h"

namespace myslam{
    class Camera{
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            typedef std::shared_ptr<Camera> Ptr;

            double fx_ = 0, fy_ = 0, cx_ = 0, cy_ = 0, baseline_ = 0; // Camera intrinsics
            SE3 pose_;     // Camera pose: T_c_w = [R|t]
            SE3 pose_inv_; // Inverse of camera pose

            Camera();

            Camera(double fx, double fy, double cx, double cy, double baseline, const SE3 &pose)
                : fx_(fx), fy_(fy), cx_(cx), cy_(cy), baseline_(baseline), pose_(pose) {
                    pose_inv_ = pose_.inverse();
                }

            SE3 Pose() const { return pose_; }

            // intrinsic matrix
            Mat33 K() const {
                Mat33 K;
                K << fx_, 0, cx_,
                     0, fy_, cy_,
                     0,  0,  1;
                return K;
            }

            // 3D -> 2D: Convert a 3D camera point to pixel coordinates.
            Vec3 world2camera(const Vec3 &p_w, const SE3 &T_c_w);

            // 2D -> 3D: Convert a 3D camera point to world coordinates.
            Vec3 camera2world(const Vec3 &p_c, const SE3 &T_c_w);

            // Project 3D camera point to 2D pixel coordinates.
            Vec2 camera2pixel(const Vec3 &p_c);

            // Reconstruct 3D camera point from pixel coordinates and depth.
            Vec3 pixel2camera(const Vec2 &p_p, double depth=1);

            // Transform pixel coordinates to world coordinates.
            Vec3 pixel2world(const Vec2 &p_p, const SE3 &T_c_w, double depth=1);

            // Project a world point to pixel coordinates.
            Vec2 world2pixel(const Vec3 &p_w, const SE3 &T_c_w);
    };
}

#endif // MYSLAM_CAMERA_H