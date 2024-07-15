#include <simulator/plain.hpp>
Plain::Plain()
{
}
Plain::~Plain()
{
  th_pos_ = 0.3 * DYNAMIC_RANGE, th_neg_ = 0.6 * DYNAMIC_RANGE, th_noise_ = 0.035 * DYNAMIC_RANGE;
}
Plain::Plain(int width, int height, sdf::ElementPtr _sdf)
{
  // set threshold
  th_pos_ = 0.3 * DYNAMIC_RANGE, th_neg_ = 0.6 * DYNAMIC_RANGE, th_noise_ = 0.035 * DYNAMIC_RANGE;
  if (_sdf->HasElement("posThreshold"))
  {
    th_pos_ = _sdf->GetElement("posThreshold")->Get<double_t>() * DYNAMIC_RANGE;
  }
  else
  {
    ROS_WARN("[DVS_plugin] No positive threshold was found, default value is 0.3.\n");
  }
  if (_sdf->HasElement("negThreshold"))
  {
    th_neg_ = _sdf->GetElement("negThreshold")->Get<double_t>() * DYNAMIC_RANGE;
  }
  else
  {
    ROS_WARN("[DVS_plugin] No negative threshold was found, default value is 0.6.\n");
  }
  if (_sdf->HasElement("noiseThreshold"))
  {
    th_noise_ = _sdf->GetElement("noiseThreshold")->Get<double_t>() * DYNAMIC_RANGE;
  }
  else
  {
    ROS_WARN("[DVS_plugin] No noise threshold was found, default value is 0.035.\n");
  }
}
void Plain::simulateMain(const cv::Mat *last_image, const cv::Mat *curr_image, std::vector<dvs_msgs::Event> *events, const ros::Time &current_time, const ros::Time &last_time)
{
  if (curr_image->size() == last_image->size())
  {

    cv::Mat pos_diff = *curr_image - *last_image;
    cv::Mat neg_diff = *last_image - *curr_image;

    cv::threshold(pos_diff, pos_diff, th_pos_, 1, cv::THRESH_BINARY);
    cv::threshold(neg_diff, neg_diff, th_neg_, 1, cv::THRESH_BINARY);

    this->fillEvents(&pos_diff, 1, events);
    this->fillEvents(&neg_diff, 0, events);
  }
  else
  {
    ROS_WARN_STREAM("Unexpected change in image size (" << last_image->size() << " -> " << curr_image->size() << "). Publishing no events for this frame change.");
  }
}

void Plain::fillEvents(const cv::Mat *mask, int polarity, std::vector<dvs_msgs::Event> *events)
{
  // findNonZero fails when there are no zeros
  // if (cv::countNonZero(*mask) != 0)
  {
    std::vector<cv::Point> locs;
    cv::findNonZero(*mask, locs);
    ros::Time ts = ros::Time::now();
    for (int i = 0; i < locs.size(); i++)
    {
      dvs_msgs::Event event;
      event.x = locs[i].x;
      event.y = locs[i].y;
      event.ts = ts;
      event.polarity = polarity;
      events->push_back(event);
    }
  }
}