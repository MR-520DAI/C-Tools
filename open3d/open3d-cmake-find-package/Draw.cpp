// ----------------------------------------------------------------------------
// -                        Open3D: www.open3d.org                            -
// ----------------------------------------------------------------------------
// The MIT License (MIT)
//
// Copyright (c) 2021 www.open3d.org
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
// IN THE SOFTWARE.
// ----------------------------------------------------------------------------

#include "./inc/Mydraw.h"

void Draw()
{                                                                                                   // (640, 480, 518.9676, 518.8751, 320.5551, 237.8842)
    open3d::camera::PinholeCameraIntrinsic cameraIntrinsic = open3d::camera::PinholeCameraIntrinsic(640, 480, 535.4, 539.2, 320.1, 247.6);
    open3d::geometry::RGBDImage RGBDImg;
    open3d::geometry::Image rgb, depth;
    // E:/c_project/learn/Open3D-master/examples/python/reconstruction_system/dataset/aobi_rgbd/maoge/color/
    open3d::io::ReadImage("color.png", rgb);
    open3d::io::ReadImage("depth.png", depth);

    RGBDImg = *open3d::geometry::RGBDImage::CreateFromColorAndDepth(
        rgb, depth, 5000, 5.0, false);
    auto pcd = open3d::geometry::PointCloud::CreateFromRGBDImage(RGBDImg,
        cameraIntrinsic, Eigen::Matrix4d::Identity(), true);

    open3d::visualization::DrawGeometries({pcd});
    return;
}

// int main(int argc, char *argv[]) {
//     if (argc == 2) {
//         std::string option(argv[1]);
//         if (option == "--skip-for-unit-test") {
//             open3d::utility::LogInfo("Skiped for unit test.");
//             return 0;
//         }
//     }

//     std::string filename = "fragment_000.ply";

//     open3d::geometry::PointCloud pcd;

//     open3d::io::ReadPointCloud(filename, pcd);
//     std::cout<<"p:"<<pcd.points_[0]<<std::endl;


//     auto sphere = open3d::geometry::TriangleMesh::CreateSphere(1.0);
//     sphere->ComputeVertexNormals();
//     sphere->PaintUniformColor({0.0, 1.0, 0.0});
//     open3d::visualization::DrawGeometries({sphere});
//     return 0;
// }
