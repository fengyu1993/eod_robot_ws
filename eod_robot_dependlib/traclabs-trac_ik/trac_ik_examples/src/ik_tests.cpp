#include <boost/date_time.hpp>
#include <trac_ik/trac_ik.hpp>
#include <ros/ros.h>
#include <kdl/chainiksolverpos_nr_jl.hpp>

double fRand(double min, double max)
{
  double f = (double) rand() / RAND_MAX;
  return min + f * (max - min);
}

void test(ros::NodeHandle& nh, double num_samples, std::string chain_start, std::string chain_end, double timeout, std::string urdf_param)
{
  double eps = 1e-5;

  TRAC_IK::TRAC_IK tracik_solver(chain_start, chain_end, urdf_param, timeout, eps);

  KDL::Chain chain;
  KDL::JntArray ll, ul;

  bool valid = tracik_solver.getKDLChain(chain);

  if (!valid)
  {
    ROS_ERROR("There was no valid KDL chain found");
    return;
  }

  valid = tracik_solver.getKDLLimits(ll, ul);

  if (!valid)
  {
    ROS_ERROR("There were no valid KDL joint limits found");
    return;
  }  

  assert(chain.getNrOfJoints() == ll.data.size());
  assert(chain.getNrOfJoints() == ul.data.size());

  ROS_INFO("Using %d joints", chain.getNrOfJoints());

  KDL::ChainFkSolverPos_recursive fk_solver(chain);
  KDL::ChainIkSolverVel_pinv vik_solver(chain); 
  KDL::ChainIkSolverPos_NR_JL kdl_solver(chain, ll, ul, fk_solver, vik_solver, 1, eps);

  KDL::JntArray nominal(chain.getNrOfJoints());

  for (uint j = 0; j < nominal.data.size(); j++)
  {
    nominal(j) = (ll(j) + ul(j)) / 2.0;
  }

  std::vector<KDL::JntArray> JointList;
  KDL::JntArray q(chain.getNrOfJoints());

  for (uint i = 0; i < num_samples; i++)
  {
    for (uint j = 0; j < ll.data.size(); j++)
    {
      q(j) = fRand(ll(j), ul(j));
    }
    JointList.push_back(q);
  }

  boost::posix_time::ptime start_time;
  boost::posix_time::time_duration diff;

  KDL::JntArray result;
  KDL::Frame end_effector_pose;
  int rc;  

  double total_time = 0;
  uint success = 0;

  ROS_INFO_STREAM("*** Testing KDL with " << num_samples << " random samples");

  for (uint i = 0; i < num_samples; i++)
  {
    fk_solver.JntToCart(JointList[i], end_effector_pose); // 计算正运动学
    double elapsed = 0;
    result = nominal; 
    start_time = boost::posix_time::microsec_clock::local_time();
    do
    {
      q = result;
      rc = kdl_solver.CartToJnt(q, end_effector_pose, result); // kdl方法计算逆运动学
      diff = boost::posix_time::microsec_clock::local_time() - start_time;
      elapsed = diff.total_nanoseconds() / 1e9;
    }
    while(rc < 0 && elapsed < timeout);
    total_time += elapsed;
    if (rc >=0)
      success++;
    if (int((double)i / num_samples * 100) % 10 == 0)
      ROS_INFO_STREAM_THROTTLE(1, int((i) / num_samples * 100) << "\% done");
  }

  ROS_INFO_STREAM("KDL found " << success << " solutions (" << 100.0 * success / num_samples << "\%) with an average of " << total_time / num_samples << " secs per sample");

  total_time = 0;
  success = 0;

  ROS_INFO_STREAM("*** Testing TRAC-IK with " << num_samples << " random samples");

  for (uint i = 0; i < num_samples; i++)
  {
    fk_solver.JntToCart(JointList[i], end_effector_pose); // 计算正运动学
    double elapsed = 0;
    start_time = boost::posix_time::microsec_clock::local_time();
    q = result;
    rc = tracik_solver.CartToJnt(q, end_effector_pose, result);
    diff = boost::posix_time::microsec_clock::local_time() - start_time;
    elapsed = diff.total_nanoseconds() / 1e9;
    total_time += elapsed;
    if (rc >=0)
      success++;
    if (int((double)i / num_samples * 100) % 10 == 0)
      ROS_INFO_STREAM_THROTTLE(1, int((i) / num_samples * 100) << "\% done");
  }

  ROS_INFO_STREAM("TRAC-IK found " << success << " solutions (" << 100.0 * success / num_samples << "\%) with an average of " << total_time / num_samples << " secs per sample");
}

void test_eod_robot(ros::NodeHandle& nh, double num_samples, std::string chain_start, std::string chain_end, double timeout, std::string urdf_param)
{
  double eps = 1e-5;

  TRAC_IK::TRAC_IK tracik_solver(chain_start, chain_end, urdf_param, timeout, eps);

  KDL::Chain chain;
  KDL::JntArray ll, ul;

  bool valid = tracik_solver.getKDLChain(chain);

  if (!valid)
  {
    ROS_ERROR("There was no valid KDL chain found");
    return;
  }

  valid = tracik_solver.getKDLLimits(ll, ul);

  if (!valid)
  {
    ROS_ERROR("There were no valid KDL joint limits found");
    return;
  }  

  KDL::ChainFkSolverPos_recursive fk_solver(chain); 

  KDL::JntArray nominal(chain.getNrOfJoints());

  for (uint j = 0; j < nominal.data.size(); j++)
  {
    nominal(j) = (ll(j) + ul(j)) / 2.0;
  }

  std::vector<KDL::JntArray> JointList;
  KDL::JntArray q(chain.getNrOfJoints());

  // right work
  q(0) = 0;       q(1) = -0.4363; q(2) = 0.7854;
  q(3) = -1.5708; q(4) = -2.7053; q(5) = 0;  

  // // left work
  // q(0) = 0;       q(1) = -2.7053; q(2) = -0.7854;
  // q(3) = -1.5708; q(4) = 2.7053; q(5) = 0;  

  // // zero
  // q(0) = 0;       q(1) = 0; q(2) = 0;
  // q(3) = 0;       q(4) = 0; q(5) = 0;  

  JointList.push_back(q);

  KDL::Frame end_effector_pose;

  fk_solver.JntToCart(JointList[0], end_effector_pose);

  std::cout << "p = " << std::endl;
  for(int i=0; i<3; i++)
  {
      std::cout << end_effector_pose.p.data[i] << ", ";
  }
  std::cout << std::endl;

  std::cout << "R = " << std::endl;
  for(int i=0; i<9; i++)
  {
      std::cout << end_effector_pose.M.data[i] << ", ";
      if ((i+1) % 3 == 0)
        std::cout << std::endl;
  }
  std::cout << std::endl;

  KDL::JntArray result;
  int rc = tracik_solver.CartToJnt(nominal, end_effector_pose, result);

  std::cout << "result = " << std::endl << result.data << std::endl;

  fk_solver.JntToCart(result, end_effector_pose);

  std::cout << "p = " << std::endl;
  for(int i=0; i<3; i++)
  {
      std::cout << end_effector_pose.p.data[i] << ", ";
  }
  std::cout << std::endl;

  std::cout << "R = " << std::endl;
  for(int i=0; i<9; i++)
  {
      std::cout << end_effector_pose.M.data[i] << ", ";
      if ((i+1) % 3 == 0)
        std::cout << std::endl;
  }
  std::cout << std::endl;
}


int main(int argc, char** argv)
{
  srand(1);
  ros::init(argc, argv, "ik_tests");
  ros::NodeHandle nh("~");

  int num_samples;
  std::string chain_start, chain_end, urdf_param;
  double timeout;

  nh.param("num_samples", num_samples, 1000);
  nh.param("chain_start", chain_start, std::string(""));
  nh.param("chain_end", chain_end, std::string(""));

  if (chain_start == "" || chain_end == "")
  {
    ROS_FATAL("Missing chain info in launch file");
    exit(-1);
  }

  nh.param("timeout", timeout, 0.005);
  nh.param("urdf_param", urdf_param, std::string("/robot_description"));

  if(num_samples < 1)
    num_samples = 1;

  test_eod_robot(nh, num_samples, chain_start, chain_end, timeout, urdf_param);

  test(nh, num_samples, chain_start, chain_end, timeout, urdf_param);


  return 0;
}