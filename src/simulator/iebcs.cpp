/*
 * simu.hpp
 *
 *  Created on: 12 Feb 2021
 *      Author: joubertd
 *
 * Modified from
 * https://github.com/neuromorphicsystems/IEBCS
 *
 */
#include <simulator/iebcs.hpp>

IEBCSim::IEBCSim()
{
  PER.resize(72);
  for (int i = -3; i < 5; i++)
  {
    for (int j = 1; j < 10; j++)
    {
      PER[(i + 3) * 9 + j - 1] = 1e6 / (j * (std::pow(10.0, i)));
    }
  }
}
IEBCSim::~IEBCSim()
{
}
dvs_msgs::Event IEBCSim::Event(uint64_t x, uint64_t y, bool polarity, uint64_t ts)
{
  dvs_msgs::Event event;
  event.x = x;
  event.y = y;
  event.polarity = polarity;
  event.ts.fromNSec(ts);
  return event;
}

IEBCSim::IEBCSim(int width, int height, sdf::ElementPtr _sdf)
{
  PER.resize(72);
  for (int i = -3; i < 5; i++)
  {
    for (int j = 1; j < 10; j++)
    {
      PER[(i + 3) * 9 + j - 1] = 1e6 / (j * (std::pow(10, i)));
    }
  }
  this->setShape(width, height);
  // set threshold
  double_t th_pos = 0.3 * DYNAMIC_RANGE, th_neg = 0.6 * DYNAMIC_RANGE, th_noise = 0.035 * DYNAMIC_RANGE;
  if (_sdf->HasElement("posThreshold"))
  {
    th_pos = _sdf->GetElement("posThreshold")->Get<double_t>() * DYNAMIC_RANGE;
  }
  else
  {
    ROS_WARN("[DVS_plugin] No positive threshold was found, default value is 0.3.\n");
  }
  if (_sdf->HasElement("negThreshold"))
  {
    th_neg = _sdf->GetElement("negThreshold")->Get<double_t>() * DYNAMIC_RANGE;
  }
  else
  {
    ROS_WARN("[DVS_plugin] No negative threshold was found, default value is 0.6.\n");
  }
  if (_sdf->HasElement("noiseThreshold"))
  {
    th_noise = _sdf->GetElement("noiseThreshold")->Get<double_t>() * DYNAMIC_RANGE;
  }
  else
  {
    ROS_WARN("[DVS_plugin] No noise threshold was found, default value is 0.035.\n");
  }
  this->setThreshold(th_pos, th_neg, th_noise);
  // set latency
  double_t lat = 100.0, jit = 30.0, ref = 100.0, tau = 1000.0;
  if (_sdf->HasElement("latency"))
  {
    lat = _sdf->GetElement("latency")->Get<double_t>();
  }
  else
  {
    ROS_WARN("[DVS_plugin] No latency was found, default value is 100 ns.\n");
  }
  if (_sdf->HasElement("jitter"))
  {
    jit = _sdf->GetElement("jitter")->Get<double_t>();
  }
  else
  {
    ROS_WARN("[DVS_plugin] No jitter was found, default value is 30 ns.\n");
  }
  if (_sdf->HasElement("refractory"))
  {
    ref = _sdf->GetElement("refractory")->Get<double_t>();
  }
  else
  {
    ROS_WARN("[DVS_plugin] No refractory was found, default value is 100 ns.\n");
  }
  if (_sdf->HasElement("tau"))
  {
    tau = _sdf->GetElement("tau")->Get<double_t>();
  }
  else
  {
    ROS_WARN("[DVS_plugin] No tau was found, default value is 1000 ns.\n");
  }
  this->setLatency(lat, jit, ref, tau);
  // set noise
  if (_sdf->HasElement("luminance") && _sdf->HasElement("noiseCache"))
  {
    std::string cache_dir = _sdf->GetElement("noiseCache")->Get<std::string>();
    std::string lumi = _sdf->GetElement("luminance")->Get<std::string>();
    std::string noise_pos_path = cache_dir + "/noise_pos_" + lumi + "lux.bin";
    std::string noise_neg_path = cache_dir + "/noise_neg_" + lumi + "lux.bin";
    double_t *distrib_pos = new double_t[50 * 50 * 72]();
    double_t *distrib_neg = new double_t[50 * 50 * 72]();
    ROS_INFO("[DVS_plugin] Loading noise from %s ...\n", noise_pos_path.c_str());
    try
    {
      std::ifstream f_pos_data(noise_pos_path, std::ios::binary | std::ios::in);
      f_pos_data.read(reinterpret_cast<char *>(distrib_pos), sizeof(double_t) * 50 * 50 * 72);
      std::ifstream f_neg_data(noise_neg_path, std::ios::binary | std::ios::in);
      f_neg_data.read(reinterpret_cast<char *>(distrib_neg), sizeof(double_t) * 50 * 50 * 72);
      f_pos_data.close();
      f_neg_data.close();
      this->initNoise(distrib_pos, distrib_neg, 50);
      ROS_INFO("[DVS_plugin] Successfully load noise files!\n");
    }
    catch (const std::exception &e)
    {
      ROS_WARN("[DVS_plugin] Error while reading noise cache, noise will be disabled.\n");
      this->disableNoise();
    }
    delete[] distrib_pos;
    delete[] distrib_neg;
  }
  else
  {
    ROS_WARN("[DVS_plugin] No noise file was set, noise will be disabled.\n");
    this->disableNoise();
  }
}
void IEBCSim::setShape(int width, int height)
{
  width_ = width;
  height_ = height;
  cur_th_pos_.resize(width * height);
  cur_th_neg_.resize(width * height);
  bgn_neg_next_.resize(width * height);
  bgn_pos_next_.resize(width * height);
  tau_p_.resize(width * height);
  last_v_.resize(width * height);
  cur_v_.resize(width * height);
  cur_ref_.resize(width * height);
  time_px_.resize(width * height);
  bgn_hist_neg_.resize(width * height * 72);
  bgn_hist_pos_.resize(width * height * 72);
}
void IEBCSim::setThreshold(double_t th_pos, double_t th_neg, double_t th_noise)
{
  th_noise_ = th_noise;
  distribution_cur_th_pos_ = std::normal_distribution<double_t>(th_pos, th_noise);
  distribution_cur_th_neg_ = std::normal_distribution<double_t>(-th_neg, th_noise);
  for (int i = 0; i < width_ * height_; i++)
  {
    cur_th_pos_.at(i) = std::max(distribution_cur_th_pos_(generator_cur_th_pos_), 0.0);
    cur_th_neg_.at(i) = std::min(0.0, distribution_cur_th_neg_(generator_cur_th_neg_));
  }
}
void IEBCSim::setLatency(double_t lat, double_t jit, double_t ref, double_t tau)
{
  m_lat_ = lat;
  m_jit_ = jit;
  ref_ = ref;
  tau_ = tau;
}
void IEBCSim::disableNoise()
{
  for (int i = 0; i < width_ * height_; i++)
  {
    bgn_neg_next_.at(i) = -1;
    bgn_pos_next_.at(i) = -1;
  }
}
void IEBCSim::initNoise(const double_t *distrib_pos, const double_t *distrib_neg, uint64_t size)
{
  std::default_random_engine generator_init_noise;
  std::uniform_int_distribution<int> distribution_index(0, size - 1);
  distribution_noise_ = std::uniform_real_distribution<double_t>(0.0, 1.0);
  int index;
  for (int i = 0; i < width_ * height_; i++)
  {
    index = distribution_index(generator_init_noise);
    for (int j = 0; j < 72; j++)
    {
      bgn_hist_pos_.at(i * 72 + j) = distrib_pos[index * 72 + j];
      bgn_hist_neg_.at(i * 72 + j) = distrib_neg[index * 72 + j];
      this->initNextNoise(i / height_, i % height_, 0);
      this->initNextNoise(i / height_, i % height_, 1);
    }
  }
}
void IEBCSim::initNextNoise(int &&x, int &&y, int &&p)
{
  double_t prob = distribution_noise_(generator_noise_);
  int position = 0;
  int id = x * height_ + y;
  if (p)
  {
    while (position < 72)
    {
      if (bgn_hist_pos_.at(id * 72 + position) >= prob)
      {
        bgn_pos_next_.at(id) = (PER[position] * distribution_noise_(generator_noise_));
        position = 72;
      }
      position++;
    }
  }
  else
  {
    while (position < 72)
    {
      if (bgn_hist_neg_.at(id * 72 + position) > prob)
      {
        bgn_neg_next_.at(id) = (PER[position] * distribution_noise_(generator_noise_));
        position = 72;
      }
      position++;
    }
  }
}
void IEBCSim::updateNextNoise(int &&x, int &&y, int &&p)
{
  double_t prob = distribution_noise_(generator_noise_);
  int position = 0;
  int id = x * height_ + y;
  if (p)
  {
    while (position < 72)
    {
      if (bgn_hist_pos_.at(id * 72 + position) >= prob)
      {
        bgn_pos_next_.at(id) += PER[position];
        position = 72;
      }
      position++;
    }
  }
  else
  {
    while (position < 72)
    {
      if (bgn_hist_neg_.at(id * 72 + position) > prob)
      {
        bgn_neg_next_.at(id) += PER[position];
        position = 72;
      }
      position++;
    }
  }
}

void IEBCSim::initImg(const double_t *img)
{
  time_ = 0;
  for (int i = 0; i < width_ * height_; i++)
  {
    if (img[i] > 0)
    {
      last_v_.at(i) = img[i];
      cur_v_.at(i) = img[i];
      tau_p_.at(i) = tau_ * 255 / img[i];
      cur_ref_.at(i) = -1;
      time_px_.at(i) = 0;
    }
  }
}

void IEBCSim::masterRst()
{
  for (int i = 0; i < width_ * height_; i++)
  {
    last_v_.at(i) = cur_v_.at(i);
    cur_ref_.at(i) = std::numeric_limits<uint64_t>::max();
  }
}

void IEBCSim::checkNoise(const uint64_t dt, std::vector<dvs_msgs::Event> *ev_pk)
{
  uint64_t next_t = time_ + dt;
  auto nb_ev_before = ev_pk->size();
  for (int i = 0; i < width_ * height_; i++)
  {
    if (bgn_pos_next_.at(i) < next_t)
    {
      ev_pk->push_back(Event(i % height_, i / height_, 1, bgn_pos_next_.at(i)));
      cur_ref_.at(i) = bgn_pos_next_.at(i);
      this->updateNextNoise(i / height_, i % height_, 1);
    }
    if (bgn_neg_next_.at(i) < next_t)
    {
      ev_pk->push_back(Event(i % height_, i / height_, 0, bgn_neg_next_.at(i)));
      cur_ref_.at(i) = bgn_neg_next_.at(i);
      this->updateNextNoise(i / height_, i % height_, 0);
    }
  }
  auto nb_ev_after = ev_pk->size();
}

void IEBCSim::clamp(double_t &val)
{
  if (val < 0)
    val = 0;
  if (val > 1e4)
    val = 1e4;
}

void IEBCSim::updateImg(const double_t *img, const uint64_t dt, std::vector<dvs_msgs::Event> *ev_pk)
{
  double_t img_l, target, amp, lat;
  uint64_t t_event;
  this->checkNoise(dt, ev_pk);
  for (int i = 0; i < width_ * height_; i++)
  {
    if (img[i] > 0)
    {
      img_l = img[i];
      tau_p_.at(i) = tau_ * 255 / img_l;
      // Update ref
      if (cur_ref_.at(i) < time_ + dt)
      {
        last_v_.at(i) = cur_v_.at(i) + (img_l - cur_v_.at(i)) * (1 - std::exp(-((double_t)(cur_ref_.at(i) - time_px_.at(i))) / tau_p_.at(i)));
        cur_v_.at(i) = last_v_.at(i);
        time_px_.at(i) = cur_ref_.at(i);
        cur_ref_.at(i) = std::numeric_limits<uint64_t>::max();
      }
      target = cur_v_.at(i) + (img_l - cur_v_.at(i)) * (1 - std::exp(-((double_t)(time_ + dt - time_px_.at(i))) / tau_p_[i]));
      // Check contrast
      while ((target - last_v_.at(i) > cur_th_pos_.at(i)) & (cur_ref_.at(i) == std::numeric_limits<uint64_t>::max()))
      {
        amp = (last_v_.at(i) + cur_th_pos_.at(i) - cur_v_.at(i)) / (img_l - cur_v_.at(i));
        distribution_lat_ = std::normal_distribution<double_t>(m_lat_ - tau_p_.at(i) * std::log(1 - amp), std::sqrt(std::pow(m_jit_, 2) + std::pow(th_noise_ * tau_p_[i] / (img_l - cur_v_[i]), 2)));
        lat = distribution_lat_(generator_lat_);
        this->clamp(lat);
        t_event = static_cast<double_t>(lat);
        ev_pk->push_back(Event(i % height_, i / height_, 1, time_px_.at(i) + t_event));
        cur_ref_.at(i) = time_px_.at(i) + t_event + ref_;
        cur_th_pos_.at(i) = std::max(0.0, (double_t)distribution_cur_th_pos_(generator_cur_th_pos_));
        if (cur_ref_.at(i) < time_ + dt)
        {
          last_v_.at(i) = cur_v_.at(i) + (img_l - cur_v_.at(i)) * (1 - std::exp(-((double_t)(cur_ref_.at(i) - time_px_.at(i))) / tau_p_.at(i)));
          cur_v_.at(i) = last_v_.at(i);
          time_px_.at(i) = cur_ref_.at(i);
          cur_ref_.at(i) = std::numeric_limits<uint64_t>::max();
        }
      }
      while ((target - last_v_.at(i) < cur_th_neg_.at(i)) & (cur_ref_.at(i) == std::numeric_limits<uint64_t>::max()))
      {
        amp = (last_v_.at(i) + cur_th_neg_.at(i) - cur_v_.at(i)) / (img_l - cur_v_.at(i));
        distribution_lat_ = std::normal_distribution<double_t>(m_lat_ - tau_p_.at(i) * std::log(1 - amp), std::sqrt(std::pow(m_jit_, 2) + std::pow(th_noise_ * tau_p_.at(i) / (img_l - cur_v_.at(i)), 2)));
        lat = distribution_lat_(generator_lat_);
        this->clamp(lat);
        t_event = static_cast<double_t>(lat);
        ev_pk->push_back(Event(i % height_, i / height_, 0, time_px_.at(i) + t_event));
        cur_ref_.at(i) = time_px_.at(i) + t_event + ref_;
        cur_th_neg_.at(i) = std::min(0.0, (double_t)distribution_cur_th_neg_(generator_cur_th_neg_));
        if (cur_ref_.at(i) < time_ + dt)
        {
          last_v_.at(i) = cur_v_.at(i) + (img_l - cur_v_.at(i)) * (1 - std::exp(-((double_t)(cur_ref_.at(i) - time_px_.at(i))) / tau_p_.at(i)));
          cur_v_.at(i) = last_v_.at(i);
          time_px_.at(i) = cur_ref_.at(i);
          cur_ref_.at(i) = std::numeric_limits<uint64_t>::max();
        }
      }
      cur_v_.at(i) = cur_v_.at(i) + (img_l - cur_v_.at(i)) * (1 - std::exp(-((double_t)(time_ + dt - time_px_.at(i))) / tau_p_.at(i)));
      time_px_.at(i) = time_ + dt;
    }
  }
  auto nb_ev_after = ev_pk->size();
  time_ += dt;
  std::sort(ev_pk->begin(), ev_pk->end(), [](const dvs_msgs::Event &a, const dvs_msgs::Event &b) -> bool
            { return a.ts.toSec() < b.ts.toSec(); });
}
void IEBCSim::simulateMain(const cv::Mat *last_iamge, const cv::Mat *curr_image, std::vector<dvs_msgs::Event> *events, const ros::Time &current_time, const ros::Time &last_time)
{
  this->initImg(last_iamge->ptr<double_t>());
  uint64_t dt_ = (current_time.toNSec() - last_time.toNSec()) / 1e3;
  this->updateImg(curr_image->ptr<double_t>(), dt_, events);
}