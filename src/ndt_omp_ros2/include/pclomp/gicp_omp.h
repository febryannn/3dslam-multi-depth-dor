#ifndef PCLOMP_GICP_OMP_H_
#define PCLOMP_GICP_OMP_H_

#include <pcl/registration/gicp.h>
#include <Eigen/Dense>

#ifdef _OPENMP
#include <omp.h>
#endif

namespace pclomp {

template <typename PointSource, typename PointTarget>
class GeneralizedIterativeClosestPoint
    : public pcl::GeneralizedIterativeClosestPoint<PointSource, PointTarget> {
public:
  using Ptr = std::shared_ptr<GeneralizedIterativeClosestPoint<PointSource, PointTarget>>;
  using PointCloudSource = pcl::PointCloud<PointSource>;
  using PointCloudTarget = pcl::PointCloud<PointTarget>;

  GeneralizedIterativeClosestPoint() : num_threads_(1) {}

  void setNumThreads(int n) {
    num_threads_ = n;
#ifdef _OPENMP
    if (n > 0) omp_set_num_threads(n);
#endif
  }

  int getNumThreads() const { return num_threads_; }

protected:
  int num_threads_;
};

}  // namespace pclomp

#endif  // PCLOMP_GICP_OMP_H_
