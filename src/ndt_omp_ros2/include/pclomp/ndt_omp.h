#ifndef PCLOMP_NDT_OMP_H_
#define PCLOMP_NDT_OMP_H_

#include <pcl/registration/ndt.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/voxel_grid.h>
#include <Eigen/Dense>

#ifdef _OPENMP
#include <omp.h>
#endif

namespace pclomp {

enum NeighborSearchMethod { KDTREE, DIRECT7, DIRECT1 };

template <typename PointSource, typename PointTarget>
class NormalDistributionsTransform
    : public pcl::NormalDistributionsTransform<PointSource, PointTarget> {
public:
  using Ptr = std::shared_ptr<NormalDistributionsTransform<PointSource, PointTarget>>;
  using PointCloudSource = pcl::PointCloud<PointSource>;
  using PointCloudTarget = pcl::PointCloud<PointTarget>;

  NormalDistributionsTransform() : num_threads_(1), search_method_(DIRECT7) {}

  void setNumThreads(int n) {
    num_threads_ = n;
#ifdef _OPENMP
    if (n > 0) omp_set_num_threads(n);
#endif
  }

  void setNeighborhoodSearchMethod(NeighborSearchMethod method) {
    search_method_ = method;
  }

  int getNumThreads() const { return num_threads_; }
  NeighborSearchMethod getNeighborhoodSearchMethod() const { return search_method_; }

protected:
  int num_threads_;
  NeighborSearchMethod search_method_;
};

}  // namespace pclomp

#endif  // PCLOMP_NDT_OMP_H_
