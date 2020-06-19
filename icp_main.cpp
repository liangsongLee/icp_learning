//
// Created by las on 2020/6/13.
//
#include <iostream>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <vector>
#include <chrono>
#include "icp_g2o.h"

#define USING_SVD
//#define USING_G2O

#ifdef USING_SVD
void icp_SVD(const pcl::PointCloud<pcl::PointXYZ>::Ptr &target,
             const pcl::PointCloud<pcl::PointXYZ>::Ptr &source,
             Eigen::Matrix4d &T);
#endif

#ifdef USING_G2O
void icp_G2O(const pcl::PointCloud<pcl::PointXYZ>::Ptr &target,
             const pcl::PointCloud<pcl::PointXYZ>::Ptr &source,
             Eigen::Matrix4d &T);

inline g2o::SE3Quat toSE3Quat(const Eigen::Matrix4d &T)
{
    Eigen::Matrix3d R = T.block<3,3>(0,0);
    Eigen::Vector3d t = T.block<3,1>(0,3);

    return g2o::SE3Quat(R,t);
}
#endif



int main(int argc, char **argv) {
    // read cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr first(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr second(
            new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile<pcl::PointXYZ>("./PCDdata/first.pcd", *first);
    pcl::io::loadPCDFile<pcl::PointXYZ>("./PCDdata/second.pcd", *second);
    pcl::PointCloud<pcl::PointXYZ>::Ptr tranformed_second(
            new pcl::PointCloud<pcl::PointXYZ>);
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();

#ifdef USING_SVD
    std::cout << "calling SVD..." << std::endl;
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    icp_SVD(first, second, T);
    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2-t1);
    std::cout << "SVD done..." << std::endl;
    std::cout << "SVD costs time: " << time_used.count() << endl;
    std::cout << "T SVD: " << T << std::endl;
#endif

#ifdef USING_G2O
    std::cout << "calling G2O..." << std::endl;
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    icp_G2O(first, second, T);
    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2-t1);
    std::cout << "G2O done..." << std::endl;
    std::cout << "G2O costs time: " << time_used.count() << endl;
    std::cout << "T G2O: " << T << std::endl;
#endif

    pcl::transformPointCloud(*second, *tranformed_second, T);

    // show result
    pcl::visualization::PCLVisualizer::Ptr viewer(
            new pcl::visualization::PCLVisualizer("viewer"));
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color1(
            first, 0, 255, 0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color2(
            tranformed_second, 255, 0, 0);
    viewer->addPointCloud<pcl::PointXYZ>(first, color1, "first");
    viewer->addPointCloud<pcl::PointXYZ>(tranformed_second, color2,
                                         "tranformed_second");
    viewer->setPointCloudRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "first");
    viewer->setPointCloudRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "tranformed_second");
    viewer->spin();

    return 0;
}

#ifdef USING_SVD
void icp_SVD(const pcl::PointCloud<pcl::PointXYZ>::Ptr &target,
             const pcl::PointCloud<pcl::PointXYZ>::Ptr &source,
             Eigen::Matrix4d &T) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr optimized_cloud(
            new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*source, *optimized_cloud);
    pcl::KdTreeFLANN<pcl::PointXYZ> tree;
    tree.setInputCloud(target);
    T.setIdentity();
    Eigen::Matrix4d init_T = Eigen::Matrix4d::Identity();
    for (int iter = 0; iter < 100; iter++) {
        std::cout << "iter number: " << iter << std::endl;
        pcl::transformPointCloud(*source, *optimized_cloud, init_T);
        Eigen::MatrixXd P(3, source->points.size());
        Eigen::MatrixXd Q(3, source->points.size());
        for (size_t i = 0; i < optimized_cloud->points.size(); ++i) {
            std::vector<int> idx;
            std::vector<float> dist;
            tree.nearestKSearch(optimized_cloud->points[i], 1, idx, dist);

            P(0, i) = source->points[i].x;
            P(1, i) = source->points[i].y;
            P(2, i) = source->points[i].z;
            Q(0, i) = target->points[idx[0]].x;
            Q(1, i) = target->points[idx[0]].y;
            Q(2, i) = target->points[idx[0]].z;
        }

        Eigen::Vector3d p_mean = P.rowwise().mean();
        Eigen::Vector3d q_mean = Q.rowwise().mean();
        Eigen::MatrixXd one_matrix(1, source->points.size());
        one_matrix.setOnes();
        auto p_means = p_mean * one_matrix;
        auto q_means = q_mean * one_matrix;
        P = P - p_means;
        Q = Q - q_means;
        Eigen::Matrix3d W = Q * P.transpose();
        Eigen::JacobiSVD<Eigen::Matrix3d> svd(W, Eigen::ComputeFullU | Eigen::ComputeFullV);
        Eigen::Matrix3d U = svd.matrixU();
        Eigen::Matrix3d V = svd.matrixV();

        Eigen::Matrix3d R = U * (V.transpose());
        if (R.determinant() < 0) {
            R = -R;
        }
        Eigen::Vector3d t = q_mean - R * p_mean;
        T.block<3, 3>(0, 0) = R;
        T.block<3, 1>(0, 3) = t;
        Eigen::Matrix4d delta_T = init_T.inverse() * T;
        if (delta_T.isIdentity(1e-4)) {
            break;
        }
        init_T = T;
    }
}
#endif

#ifdef USING_G2O
void icp_G2O(const pcl::PointCloud<pcl::PointXYZ>::Ptr &target,
             const pcl::PointCloud<pcl::PointXYZ>::Ptr &source,
             Eigen::Matrix4d &T){
    pcl::PointCloud<pcl::PointXYZ>::Ptr optimized_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*source, *optimized_cloud);
    pcl::KdTreeFLANN<pcl::PointXYZ> tree;
    tree.setInputCloud(target);

    T.setIdentity();
    Eigen::Matrix4d init_T = Eigen::Matrix4d::Identity();

    for (int iter = 0; iter < 100; iter++) {
        std::cout << "iter number: " << iter << std::endl;
        g2o::SparseOptimizer optimizer;
        g2o::BlockSolverX::LinearSolverType *linearSolver;

        linearSolver = new g2o::LinearSolverDense<g2o::BlockSolverX::PoseMatrixType>();

        g2o::BlockSolverX *solver_ptr = new g2o::BlockSolverX(linearSolver);

        g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
        optimizer.setAlgorithm(solver);
//        optimizer.setVerbose(true);

        pcl::transformPointCloud(*source, *optimized_cloud, init_T);
        g2o::SE3Quat T_ = toSE3Quat(init_T);
        g2o::VertexSE3Expmap *pose = new g2o::VertexSE3Expmap();
        pose->setId(0);
        pose->setEstimate(T_);
        optimizer.addVertex(pose);

        for (size_t i = 0; i < optimized_cloud->points.size(); ++i) {
            std::vector<int> idx;
            std::vector<float> dist;
            tree.nearestKSearch(optimized_cloud->points[i], 1, idx, dist);
            if (dist[0] > 5.)
                continue;

            Eigen::Vector3d source_point(source->points[i].x, source->points[i].y,
                                         source->points[i].z);
            Eigen::Vector3d target_point(target->points[idx[0]].x,
                                         target->points[idx[0]].y,
                                         target->points[idx[0]].z);

            g2o::EdgeProjectXYZPoseOnlyICP *edge = new g2o::EdgeProjectXYZPoseOnlyICP();
            edge->point_ = source_point;
            edge->setVertex(0, pose);
            edge->setMeasurement(target_point);
            edge->setInformation(Eigen::Matrix3d::Identity());
            optimizer.addEdge(edge);
        }

        optimizer.initializeOptimization();
        optimizer.optimize(10);

        Eigen::Matrix3d R = pose->estimate().rotation().toRotationMatrix();
        Eigen::Vector3d t = pose->estimate().translation();
        T.block<3,3>(0,0) = R;
        T.block<3,1>(0,3) = t;
        Eigen::Matrix4d delta_T = init_T.inverse() * T;
        if (delta_T.isIdentity(1e-4)) {
            break;
        }
        init_T = T;
    }
}
#endif