
#include <fuse_core/uuid.h>
#include <locus_cpp/timing.h>
#include <ros/time.h>

#include <boost/bimap/bimap.hpp>
#include <boost/bimap/multiset_of.hpp>
#include <boost/bimap/unordered_set_of.hpp>

#include <random>
#include <unordered_map>
#include <utility>
#include <vector>


constexpr size_t ELEMENT_COUNT = 100;
constexpr size_t TRIALS = 1000;


using MapIndex = std::unordered_map<fuse_core::UUID, ros::Time, fuse_core::uuid::hash>;
using BoostIndex = boost::bimaps::bimap<boost::bimaps::multiset_of<ros::Time>,
                                        boost::bimaps::unordered_set_of<fuse_core::UUID>>;


int main(int argc, char **argv)
{
  auto map_index = MapIndex();
  auto boost_index = BoostIndex();

  std::default_random_engine generator;

  for (size_t trial = 0; trial < TRIALS; ++trial)
  {
    // Create a set of random elements
    std::uniform_real_distribution<double> distribution(0.0, 1000.0);
    std::vector<std::pair<fuse_core::UUID, ros::Time>> elements;
    for (size_t i = 0; i < ELEMENT_COUNT; ++i)
    {
      elements.push_back(std::make_pair(fuse_core::uuid::generate(), ros::Time(distribution(generator))));
    }
    auto query_time = ros::Time(distribution(generator));

    TIC(map_insert);
    for (const auto& element : elements)
    {
      map_index.insert(element);
    }
    TOC(map_insert);

    auto map_results = std::vector<fuse_core::UUID>();
    TIC(map_find);
    for (const auto& element : map_index)
    {
      if (element.second < query_time)
      {
        map_results.push_back(element.first);
      }
    }
    TOC(map_find);

    TIC(boost_insert);
    for (const auto& element : elements)
    {
      boost_index.right.insert(element);
    }
    TOC(boost_insert);

    auto boost_results = std::vector<fuse_core::UUID>();
    TIC(boost_find);
    for (const auto& element : boost_index.left)
    {
      if (element.first >= query_time) break;
      boost_results.push_back(element.second);
    }
    TOC(boost_find);
  }

  std::cout << "*** ELEMENT_COUNT = " << ELEMENT_COUNT << std::endl;
  TICTOC_PRINT();

  return 0;
}
