//
// Created by glawless on 23.05.17.
//

#ifndef TARGET_TRACKER_DISTRIBUTED_KF_DISTRIBUTEDKF3D_H
#define TARGET_TRACKER_DISTRIBUTED_KF_DISTRIBUTEDKF3D_H

#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/StdDeque>
#include <vector>
#include <deque>
#include <string>
#include <dynamic_reconfigure/server.h>
#include <target_tracker_distributed_kf/KalmanFilterParamsConfig.h>
#include <uav_msgs/uav_pose.h>
#include <memory>

namespace target_tracker_distributed_kf {

  using namespace geometry_msgs;
  using namespace std;
  using namespace Eigen;

  typedef struct CacheElement_s{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    ros::Time stamp;
    string frame_id;
    VectorXd state;
    MatrixXd cov;
    vector<shared_ptr<PoseWithCovariance>> measurements;
    bool isSelfRobot;

    CacheElement_s() = delete; // at least use constructor with stamp
    CacheElement_s(const std_msgs::Header& h, const int vecSize, const bool selfRobotFlag) :
      stamp(h.stamp), frame_id(h.frame_id), state(VectorXd::Zero(vecSize)), cov(MatrixXd::Zero(vecSize, vecSize)), measurements(0), isSelfRobot(selfRobotFlag){
      measurements.reserve(5);
    };
    CacheElement_s(const int vecSize, const PoseWithCovarianceStamped& m, const bool selfRobotFlag) :
      stamp(m.header.stamp), frame_id(m.header.frame_id), state(VectorXd::Zero(vecSize)), cov(MatrixXd::Zero(vecSize, vecSize)), measurements(0), isSelfRobot(selfRobotFlag){
      measurements.reserve(5);
      auto ptr = shared_ptr<PoseWithCovariance>(new PoseWithCovariance);
      ptr->pose = m.pose.pose;
      ptr->covariance = m.pose.covariance;
      measurements.push_back(ptr);
    };

  }CacheElement;

  class Cache : public deque<CacheElement, aligned_allocator<CacheElement> >{

  private:
      size_t cache_size_{0};

  public:
    Cache() : cache_size_(0) {};

    void set_cache_size(size_t sz) {
      this->cache_size_ = sz;
    }

    deque<CacheElement>::iterator insert_ordered(const CacheElement& elem){
      // Sanity check
      if(cache_size_ <= 0)
        return end();

      // First element, special case
      if(empty())
        return insert(begin(), elem);

      // If full and time lower than the first in deque, then return nullptr to signalize that it failed to insert
      if(size() == cache_size_ && begin()->stamp > elem.stamp)
        return end();

      // Pop enough elements to keep at max size
      while(!empty() && (size() + 1 > cache_size_))
        pop_front();

      if(empty())
        return insert(begin(), elem);

      // Always start from end, can't however use rbegin since we need to insert
      auto it = end();

      // Look for the stamp just after elem's stamp
      for(; it != begin(); --it) {
        if(it->stamp == elem.stamp){
          // Two measurements for the same time, in this case we don't insert and instead add to the vector of measurements
          if(!elem.measurements.empty()) {
            auto ptr = shared_ptr<PoseWithCovariance>(new PoseWithCovariance);
            ptr->pose = elem.measurements[0]->pose;
            ptr->covariance = elem.measurements[0]->covariance;
            it->measurements.push_back(ptr);
          }
          return it;
        }
        else if((it-1)->stamp < elem.stamp) {
          // Check against previous one because insertion iterator should point to position after where we want to insert
          break;
        }
      }

      // Insert is done before the it position
      it = insert(it, elem);

      // Return the iterator to inserted position
      return it;
    }

    void print(std::ostream &stream) const{
      stream << "Cache at time " << ros::Time::now() << std::endl;
      if(empty())
        stream << "Empty" << std::endl;
      else {
        stream << "Total of " << size() << " elements" << std::endl;
        stream << "\tStamp \t\t\t#Measurements\tFrame\t\tState" << std::endl;
      }

      IOFormat NoEndLineOnRowSep(StreamPrecision,0," ", " ");
      for(auto it=begin(); it != end(); ++it){
        stream << "\t" << it->stamp << "\t" << it->measurements.size() << "\t" << it->frame_id << it->state.format(NoEndLineOnRowSep) << std::endl;
      }
    }
  };

  inline std::ostream& operator<< (std::ostream& stream, const Cache& cache) {
    cache.print(stream);
    return stream;
  }

  class DistributedKF3D {

  private:
    ros::NodeHandle nh_, pnh_;
    std::vector<std::unique_ptr<ros::Subscriber>> other_subs_;
    ros::Subscriber pose_sub_;
    ros::Subscriber self_sub_;
    Cache state_cache_;
    ros::Publisher targetPub_;
    ros::Publisher offsetPub_;
    geometry_msgs::PoseWithCovarianceStamped msg_;
    dynamic_reconfigure::Server<KalmanFilterParamsConfig> dyn_rec_server_;

    void initializeFilter();
    void setUnknownInitial(CacheElement&);
    bool predict(const CacheElement&, CacheElement&);
    bool update(CacheElement&);
    void initializeStaticMatrices();
    void populateJacobianG(MatrixXd &G, const double deltaT);
    void populateJacobianQ(MatrixXd &Q, const PoseWithCovariance& pcov);
    bool detectBackwardsTimeJump();

    void predictAndPublish(const uav_msgs::uav_poseConstPtr&);
    void publishStateAndCov(const CacheElement&);

  protected:
    static constexpr auto state_size = 9;
    static constexpr auto measurement_state_size = 6;
    double time_threshold{10.0};

  public:
    DistributedKF3D();

    void selfMeasurementCallback(const PoseWithCovarianceStamped&);
    void otherMeasurementCallback(const PoseWithCovarianceStamped&);
    void measurementsCallback(const PoseWithCovarianceStamped&, bool);
    void dynamicReconfigureCallback(KalmanFilterParamsConfig &config, uint32_t level);

    // Static matrices
    MatrixXd I, Hself, Hother, R;

    // Initial values for uncertainty on reset
    double  initialUncertaintyPosXY{100},
            initialUncertaintyPosZ{10},
            initialUncertaintyVelXY{1},
            initialUncertaintyVelZ{.5},
            initialUncertaintyOffsetXY{1},
            initialUncertaintyOffsetZ{3};

    // Noises
    double  noisePosXVar{0.01},
            noiseVelXVar{0.01},
            noiseOffXVar{0.01},
            noisePosYVar{0.01},
            noiseVelYVar{0.01},
            noiseOffYVar{0.01},
            noisePosZVar{0.01},
            noiseVelZVar{0.01},
            noiseOffZVar{0.01};

    // Decay of velocity
    double velocityDecayTime{3.0};
    double offsetDecayTime{30.0};

      void initializeSubscribers();
  };

}

#endif //TARGET_TRACKER_DISTRIBUTED_KF_DISTRIBUTEDKF3D_H
