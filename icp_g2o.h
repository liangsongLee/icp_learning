//
// Created by las on 2020/6/13.
//

#ifndef ICP_HW_ICP_G2O_H
#define ICP_HW_ICP_G2O_H

#include "3rdParty/g2o/g2o/core/base_vertex.h"
#include "3rdParty/g2o/g2o/core/base_unary_edge.h"
#include "3rdParty/g2o/g2o/core/block_solver.h"
#include "3rdParty/g2o/g2o/types/types_six_dof_expmap.h"
#include "3rdParty/g2o/g2o/core/optimization_algorithm_levenberg.h"
#include "3rdParty/g2o/g2o/solvers/linear_solver_dense.h"

namespace g2o {

    class EdgeProjectXYZPoseOnlyICP : public g2o::BaseUnaryEdge<3, Eigen::Vector3d, VertexSE3Expmap> {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

        EdgeProjectXYZPoseOnlyICP();

        virtual void computeError() override {
            const VertexSE3Expmap *pose = static_cast<const VertexSE3Expmap *> ( _vertices[0] );
            _error = _measurement - pose->estimate().map(point_);
        }

        virtual void linearizeOplus();

        bool read(istream &in);

        bool write(ostream &out) const;

        Eigen::Vector3d point_;
    };

}

#endif //ICP_HW_ICP_G2O_H