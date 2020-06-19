//
// Created by las on 2020/6/13.
//

#include "icp_g2o.h"

namespace g2o{
    EdgeProjectXYZPoseOnlyICP::EdgeProjectXYZPoseOnlyICP() : BaseUnaryEdge<3, Vector3d, VertexSE3Expmap>() {}
    bool EdgeProjectXYZPoseOnlyICP::read(std::istream& in){
        for (int i=0; i<3; i++){
            in >> _measurement[i];
        }
        for (int i=0; i<3; i++)
            for (int j=i; j<3; j++) {
                in >> information()(i,j);
                if (i!=j)
                    information()(j,i)=information()(i,j);
            }
        return true;
    }

    bool EdgeProjectXYZPoseOnlyICP::write(std::ostream& out) const {

        for (int i=0; i<3; i++){
            out << measurement()[i] << " ";
        }

        for (int i=0; i<3; i++)
            for (int j=i; j<3; j++){
                out << " " <<  information()(i,j);
            }
        return out.good();
    }

    void EdgeProjectXYZPoseOnlyICP::linearizeOplus() {
        VertexSE3Expmap *vi = static_cast<VertexSE3Expmap *>(_vertices[0]);
        SE3Quat T(vi->estimate());

        Vector3d xyz_trans = T.map(point_);

        _jacobianOplusXi.block<3, 3>(0, 0) = skew(xyz_trans);
        _jacobianOplusXi.block<3, 3>(0, 3) = -Eigen::Matrix3d::Identity();
    }
}