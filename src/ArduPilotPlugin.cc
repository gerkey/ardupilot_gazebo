/* * Copyright (C) 2016 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
#include <functional>
#include <fcntl.h>
#ifdef _WIN32
  #include <Winsock2.h>
  #include <Ws2def.h>
  #include <Ws2ipdef.h>
  #include <Ws2tcpip.h>
  using raw_type = char;
#else
  #include <sys/socket.h>
  #include <netinet/in.h>
  #include <netinet/tcp.h>
  #include <arpa/inet.h>
  #include <unistd.h>
  using raw_type = void;
#endif

#if defined(_MSC_VER)
  #include <BaseTsd.h>
  typedef SSIZE_T ssize_t;
#endif

#include <chrono>
#include <mutex>
#include <string>
#include <vector>

#include "include/ArduPilotPlugin.hh"

#include <ignition/gazebo/components/AngularVelocity.hh>
#include <ignition/gazebo/components/Imu.hh>
#include <ignition/gazebo/components/JointForceCmd.hh>
#include <ignition/gazebo/components/JointPosition.hh>
#include <ignition/gazebo/components/JointVelocity.hh>
#include <ignition/gazebo/components/JointVelocityCmd.hh>
#include <ignition/gazebo/components/LinearAcceleration.hh>
#include <ignition/gazebo/components/LinearVelocity.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/ParentEntity.hh>
#include <ignition/gazebo/components/Pose.hh>

#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/Types.hh>
#include <ignition/gazebo/Util.hh>

#include <ignition/math/Filter.hh>
#include <ignition/math/Helpers.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/PID.hh>
#include <ignition/math/Vector3.hh>

#include <ignition/msgs/imu.pb.h>

#include <ignition/plugin/Register.hh>

#include <ignition/transport/Node.hh>

#include <sdf/sdf.hh>

#define MAX_MOTORS 255

//GZ_REGISTER_MODEL_PLUGIN(ArduPilotPlugin)

/// \brief A servo packet.
struct ServoPacket
{
  /// \brief Motor speed data.
  /// should rename to servo_command here and in ArduPilot SIM_Gazebo.cpp
  float motorSpeed[MAX_MOTORS] = {0.0f};
};

/// \brief Flight Dynamics Model packet that is sent back to the ArduPilot
struct fdmPacket
{
  /// \brief packet timestamp
  double timestamp;

  /// \brief IMU angular velocity
  double imuAngularVelocityRPY[3];

  /// \brief IMU linear acceleration
  double imuLinearAccelerationXYZ[3];

  /// \brief IMU quaternion orientation
  double imuOrientationQuat[4];

  /// \brief Model velocity in NED frame
  double velocityXYZ[3];

  /// \brief Model position in NED frame
  double positionXYZ[3];
/*  NOT MERGED IN MASTER YET
  /// \brief Model latitude in WGS84 system
  double latitude = 0.0;
  
  /// \brief Model longitude in WGS84 system
  double longitude = 0.0;
  
  /// \brief Model altitude from GPS
  double altitude = 0.0;
  
  /// \brief Model estimated from airspeed sensor (e.g. Pitot) in m/s
  double airspeed = 0.0; 
  
  /// \brief Battery voltage. Default to -1 to use sitl estimator.
  double battery_voltage = -1.0;
  
  /// \brief Battery Current.
  double battery_current = 0.0;
  
  /// \brief Model rangefinder value. Default to -1 to use sitl rangefinder.
  double rangefinder = -1.0;
*/
};

/// \brief Control class
class Control
{
  /// \brief Constructor
  public: Control()
  {
    // most of these coefficients are not used yet.
    this->rotorVelocitySlowdownSim = this->kDefaultRotorVelocitySlowdownSim;
    this->frequencyCutoff = this->kDefaultFrequencyCutoff;
    this->samplingRate = this->kDefaultSamplingRate;

    this->pid.Init(0.1, 0, 0, 0, 0, 1.0, -1.0);
  }

  /// \brief control id / channel
  public: int channel = 0;

  /// \brief Next command to be applied to the propeller
  public: double cmd = 0;

  /// \brief Velocity PID for motor control
  public: ignition::math::PID pid;

  /// \brief Control type. Can be:
  /// VELOCITY control velocity of joint
  /// POSITION control position of joint
  /// EFFORT control effort of joint
  public: std::string type;
  
  /// \brief use force controler
  public: bool useForce = true;

  /// \brief Control propeller joint.
  public: std::string jointName;

  /// \brief Control propeller joint.
  public: ignition::gazebo::Entity joint;

  /// \brief direction multiplier for this control
  public: double multiplier = 1;

  /// \brief input command offset
  public: double offset = 0;

  /// \brief unused coefficients
  public: double rotorVelocitySlowdownSim;
  public: double frequencyCutoff;
  public: double samplingRate;
  public: ignition::math::OnePole<double> filter;

  public: static double kDefaultRotorVelocitySlowdownSim;
  public: static double kDefaultFrequencyCutoff;
  public: static double kDefaultSamplingRate;
};

double Control::kDefaultRotorVelocitySlowdownSim = 10.0;
double Control::kDefaultFrequencyCutoff = 5.0;
double Control::kDefaultSamplingRate = 0.2;

// Private data class
class ignition::gazebo::systems::ArduPilotSocketPrivate
{
  /// \brief constructor
  public: ArduPilotSocketPrivate()
  {
    // initialize socket udp socket
    fd = socket(AF_INET, SOCK_DGRAM, 0);
    #ifndef _WIN32
    // Windows does not support FD_CLOEXEC
    fcntl(fd, F_SETFD, FD_CLOEXEC);
    #endif
  }

  /// \brief destructor
  public: ~ArduPilotSocketPrivate()
  {
    if (fd != -1)
    {
      ::close(fd);
      fd = -1;
    }
  }

  /// \brief Bind to an adress and port
  /// \param[in] _address Address to bind to.
  /// \param[in] _port Port to bind to.
  /// \return True on success.
  public: bool Bind(const char *_address, const uint16_t _port)
  {
    struct sockaddr_in sockaddr;
    this->MakeSockAddr(_address, _port, sockaddr);

    if (bind(this->fd, (struct sockaddr *)&sockaddr, sizeof(sockaddr)) != 0)
    {
      shutdown(this->fd, 0);
      #ifdef _WIN32
      closesocket(this->fd);
      #else
      close(this->fd);
      #endif
      return false;
    }
    int one = 1;
    setsockopt(this->fd, SOL_SOCKET, SO_REUSEADDR,
        reinterpret_cast<const char *>(&one), sizeof(one));

    #ifdef _WIN32
    u_long on = 1;
    ioctlsocket(this->fd, FIONBIO,
              reinterpret_cast<u_long FAR *>(&on));
    #else
    fcntl(this->fd, F_SETFL,
        fcntl(this->fd, F_GETFL, 0) | O_NONBLOCK);
    #endif
    return true;
  }

  /// \brief Connect to an adress and port
  /// \param[in] _address Address to connect to.
  /// \param[in] _port Port to connect to.
  /// \return True on success.
  public : bool Connect(const char *_address, const uint16_t _port)
  {
    struct sockaddr_in sockaddr;
    this->MakeSockAddr(_address, _port, sockaddr);

    if (connect(this->fd, (struct sockaddr *)&sockaddr, sizeof(sockaddr)) != 0)
    {
      shutdown(this->fd, 0);
      #ifdef _WIN32
      closesocket(this->fd);
      #else
      close(this->fd);
      #endif
      return false;
    }
    int one = 1;
    setsockopt(this->fd, SOL_SOCKET, SO_REUSEADDR,
        reinterpret_cast<const char *>(&one), sizeof(one));

    #ifdef _WIN32
    u_long on = 1;
    ioctlsocket(this->fd, FIONBIO,
              reinterpret_cast<u_long FAR *>(&on));
    #else
    fcntl(this->fd, F_SETFL,
        fcntl(this->fd, F_GETFL, 0) | O_NONBLOCK);
    #endif
    return true;
  }

  /// \brief Make a socket
  /// \param[in] _address Socket address.
  /// \param[in] _port Socket port
  /// \param[out] _sockaddr New socket address structure.
  public: void MakeSockAddr(const char *_address, const uint16_t _port,
    struct sockaddr_in &_sockaddr)
  {
    memset(&_sockaddr, 0, sizeof(_sockaddr));

    #ifdef HAVE_SOCK_SIN_LEN
      _sockaddr.sin_len = sizeof(_sockaddr);
    #endif

    _sockaddr.sin_port = htons(_port);
    _sockaddr.sin_family = AF_INET;
    _sockaddr.sin_addr.s_addr = inet_addr(_address);
  }

  public: ssize_t Send(const void *_buf, size_t _size)
  {
    return send(this->fd, _buf, _size, 0);
  }

  /// \brief Receive data
  /// \param[out] _buf Buffer that receives the data.
  /// \param[in] _size Size of the buffer.
  /// \param[in] _timeoutMS Milliseconds to wait for data.
  public: ssize_t Recv(void *_buf, const size_t _size, uint32_t _timeoutMs)
  {
    fd_set fds;
    struct timeval tv;

    FD_ZERO(&fds);
    FD_SET(this->fd, &fds);

    tv.tv_sec = _timeoutMs / 1000;
    tv.tv_usec = (_timeoutMs % 1000) * 1000UL;

    if (select(this->fd+1, &fds, NULL, NULL, &tv) != 1)
    {
        return -1;
    }

    #ifdef _WIN32
    return recv(this->fd, reinterpret_cast<char *>(_buf), _size, 0);
    #else
    return recv(this->fd, _buf, _size, 0);
    #endif
  }

  /// \brief Socket handle
  private: int fd;
};

// Private data class
class ignition::gazebo::systems::ArduPilotPluginPrivate
{
  /// \brief Pointer to the update event connection.
  //public: event::ConnectionPtr updateConnection;

  /// \brief Pointer to the model;
  public: ignition::gazebo::Entity entity{ignition::gazebo::kNullEntity};
  public: ignition::gazebo::Model model{ignition::gazebo::kNullEntity};

  /// \brief String of the model name;
  public: std::string modelName;

  /// \brief array of propellers
  public: std::vector<Control> controls;

  /// \brief keep track of controller update sim-time.
  public: std::chrono::steady_clock::duration lastControllerUpdateTime{0};

  /// \brief Controller update mutex.
  public: std::mutex mutex;

  /// \brief Ardupilot Socket for receive motor command on gazebo
  public: ignition::gazebo::systems::ArduPilotSocketPrivate socket_in;

  /// \brief Ardupilot Socket to send state to Ardupilot
  public: ignition::gazebo::systems::ArduPilotSocketPrivate socket_out;

  /// \brief Ardupilot address
  public: std::string fdm_addr;

  /// \brief The Ardupilot listen address
  public: std::string listen_addr;

  /// \brief Ardupilot port for receiver socket
  public: uint16_t fdm_port_in;

  /// \brief Ardupilot port for sender socket
  public: uint16_t fdm_port_out;

  /// \brief Pointer to an IMU sensor
  //public: std::unique_ptr<ignition::sensors::ImuSensor> imuSensor;

  public: bool imuInitialized{false};
  public: std::string imuTopicName;
  public: ignition::transport::Node node;
  public: void imuCb(const ignition::msgs::IMU &_msg)
  {
    ignerr << "Got IMU data: " << _msg.DebugString() << std::endl;
  }
  
  /// \brief Pointer to an GPS sensor
  //public: ignition::sensors::GpsSensor gpsSensor;
  
  /// \brief Pointer to an Rangefinder sensor
  //public: ignition::sensors::RaySensor rangefinderSensor;

  /// \brief false before ardupilot controller is online
  /// to allow gazebo to continue without waiting
  public: bool arduPilotOnline;

  /// \brief number of times ArduCotper skips update
  public: int connectionTimeoutCount;

  /// \brief number of times ArduCotper skips update
  /// before marking ArduPilot offline
  public: int connectionTimeoutMaxCount;
};


/////////////////////////////////////////////////
ignition::gazebo::systems::ArduPilotPlugin::ArduPilotPlugin()
  : dataPtr(new ArduPilotPluginPrivate)
{
  this->dataPtr->arduPilotOnline = false;
  this->dataPtr->connectionTimeoutCount = 0;
}

/////////////////////////////////////////////////
ignition::gazebo::systems::ArduPilotPlugin::~ArduPilotPlugin()
{
}

/////////////////////////////////////////////////
//void ArduPilotPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
void ignition::gazebo::systems::ArduPilotPlugin::Configure(const ignition::gazebo::Entity &_entity,
      const std::shared_ptr<const sdf::Element> &_sdf,
      ignition::gazebo::EntityComponentManager &_ecm,
      ignition::gazebo::EventManager &/*_eventMgr*/)
{
  this->dataPtr->entity = _entity;
  this->dataPtr->model = ignition::gazebo::Model(_entity);

  // Make a clone so that we can call non-const methods
  auto sdfClone = _sdf->Clone();

  if (!this->dataPtr->model.Valid(_ecm))
  {
    ignerr << "ArduPilotPlugin should be attached to a model "
      << "entity. Failed to initialize." << std::endl;
    return;
  }

  this->dataPtr->modelName = this->dataPtr->model.Name(_ecm);

  // modelXYZToAirplaneXForwardZDown brings us from gazebo model frame:
  // x-forward, y-right, z-down
  // to the aerospace convention: x-forward, y-left, z-up
  this->modelXYZToAirplaneXForwardZDown =
    ignition::math::Pose3d(0, 0, 0, 0, 0, 0);
  if (sdfClone->HasElement("modelXYZToAirplaneXForwardZDown"))
  {
    this->modelXYZToAirplaneXForwardZDown =
        sdfClone->Get<ignition::math::Pose3d>("modelXYZToAirplaneXForwardZDown");
  }

  // gazeboXYZToNED: from gazebo model frame: x-forward, y-right, z-down
  // to the aerospace convention: x-forward, y-left, z-up
  this->gazeboXYZToNED = ignition::math::Pose3d(0, 0, 0, IGN_PI, 0, 0);
  if (sdfClone->HasElement("gazeboXYZToNED"))
  {
    this->gazeboXYZToNED = sdfClone->Get<ignition::math::Pose3d>("gazeboXYZToNED");
  }

  // per control channel
  sdf::ElementPtr controlSDF;
  if (sdfClone->HasElement("control"))
  {
    controlSDF = sdfClone->GetElement("control");
  }
  else if (sdfClone->HasElement("rotor"))
  {
    ignwarn << "[" << this->dataPtr->modelName << "] "
           << "please deprecate <rotor> block, use <control> block instead.\n";
    controlSDF = sdfClone->GetNextElement("rotor");
  }

  while (controlSDF)
  {
    Control control;

    if (controlSDF->HasAttribute("channel"))
    {
      control.channel =
        atoi(controlSDF->GetAttribute("channel")->GetAsString().c_str());
    }
    else if (controlSDF->HasAttribute("id"))
    {
      ignwarn << "[" << this->dataPtr->modelName << "] "
             <<  "please deprecate attribute id, use channel instead.\n";
      control.channel =
        atoi(controlSDF->GetAttribute("id")->GetAsString().c_str());
    }
    else
    {
      control.channel = this->dataPtr->controls.size();
      ignwarn << "[" << this->dataPtr->modelName << "] "
             <<  "id/channel attribute not specified, use order parsed ["
             << control.channel << "].\n";
    }

    if (controlSDF->HasElement("type"))
    {
      control.type = controlSDF->Get<std::string>("type");
    }
    else
    {
      ignerr << "[" << this->dataPtr->modelName << "] "
            <<  "Control type not specified,"
            << " using velocity control by default.\n";
      control.type = "VELOCITY";
    }

    if (control.type != "VELOCITY" &&
        control.type != "POSITION" &&
        control.type != "EFFORT")
    {
      ignwarn << "[" << this->dataPtr->modelName << "] "
             << "Control type [" << control.type
             << "] not recognized, must be one of VELOCITY, POSITION, EFFORT."
             << " default to VELOCITY.\n";
      control.type = "VELOCITY";
    }
    
    if (controlSDF->HasElement("useForce"))
    {
      control.useForce = controlSDF->Get<bool>("useForce");
    }

    if (controlSDF->HasElement("jointName"))
    {
      control.jointName = controlSDF->Get<std::string>("jointName");
    }
    else
    {
      ignerr << "[" << this->dataPtr->modelName << "] "
            << "Please specify a jointName,"
            << " where the control channel is attached.\n";
    }

    // Get the pointer to the joint.
    //control.joint = _model->GetJoint(control.jointName);
    control.joint = this->dataPtr->model.JointByName(_ecm, control.jointName);
    if (control.joint == ignition::gazebo::kNullEntity)
    {
      ignerr << "[" << this->dataPtr->modelName << "] "
            << "Couldn't find specified joint ["
            << control.jointName << "]. This plugin will not run.\n";
      return;
    }

    if (controlSDF->HasElement("multiplier"))
    {
      // overwrite turningDirection, deprecated.
      control.multiplier = controlSDF->Get<double>("multiplier");
    }
    else if (controlSDF->HasElement("turningDirection"))
    {
      ignwarn << "[" << this->dataPtr->modelName << "] "
             << "<turningDirection> is deprecated. Please use"
             << " <multiplier>. Map 'cw' to '-1' and 'ccw' to '1'.\n";
      std::string turningDirection = controlSDF->Get<std::string>(
          "turningDirection");
      // special cases mimic from controls_gazebo_plugins
      if (turningDirection == "cw")
      {
        control.multiplier = -1;
      }
      else if (turningDirection == "ccw")
      {
        control.multiplier = 1;
      }
      else
      {
        igndbg << "[" << this->dataPtr->modelName << "] "
              << "not string, check turningDirection as float\n";
        control.multiplier = controlSDF->Get<double>("turningDirection");
      }
    }
    else
    {
      igndbg << "[" << this->dataPtr->modelName << "] "
            << "<multiplier> (or deprecated <turningDirection>) not specified,"
            << " Default 1 (or deprecated <turningDirection> 'ccw').\n";
      control.multiplier = 1;
    }

    if (controlSDF->HasElement("offset"))
    {
      control.offset = controlSDF->Get<double>("offset");
    }
    else
    {
      igndbg << "[" << this->dataPtr->modelName << "] "
            << "<offset> not specified, default to 0.\n";
      control.offset = 0;
    }

    control.rotorVelocitySlowdownSim =
        controlSDF->Get("rotorVelocitySlowdownSim", 1).first;

    if (ignition::math::equal(control.rotorVelocitySlowdownSim, 0.0))
    {
      ignwarn << "[" << this->dataPtr->modelName << "] "
             << "control for joint [" << control.jointName
             << "] rotorVelocitySlowdownSim is zero,"
             << " assume no slowdown.\n";
      control.rotorVelocitySlowdownSim = 1.0;
    }

    control.frequencyCutoff =
          controlSDF->Get("frequencyCutoff", control.frequencyCutoff).first;
    control.samplingRate =
          controlSDF->Get("samplingRate", control.samplingRate).first;

    // use gazebo::math::Filter
    control.filter.Fc(control.frequencyCutoff, control.samplingRate);

    // initialize filter to zero value
    control.filter.Set(0.0);

    // note to use this filter, do
    // stateFiltered = filter.Process(stateRaw);

    // Overload the PID parameters if they are available.
    double param;
    // carry over from ArduCopter plugin
    param = controlSDF->Get("vel_p_gain", control.pid.PGain()).first;
    control.pid.SetPGain(param);

    param = controlSDF->Get("vel_i_gain", control.pid.IGain()).first;
    control.pid.SetIGain(param);

    param = controlSDF->Get("vel_d_gain", control.pid.DGain()).first;
    control.pid.SetDGain(param);

    param = controlSDF->Get("vel_i_max", control.pid.IMax()).first;
    control.pid.SetIMax(param);

    param = controlSDF->Get("vel_i_min", control.pid.IMin()).first;
    control.pid.SetIMin(param);

    param = controlSDF->Get("vel_cmd_max", control.pid.CmdMax()).first;
    control.pid.SetCmdMax(param);

    param = controlSDF->Get("vel_cmd_min", control.pid.CmdMin()).first;
    control.pid.SetCmdMin(param);

    // new params, overwrite old params if exist
    param = controlSDF->Get("p_gain", control.pid.PGain()).first;
    control.pid.SetPGain(param);

    param = controlSDF->Get("i_gain", control.pid.IGain()).first;
    control.pid.SetIGain(param);

    param = controlSDF->Get("d_gain", control.pid.DGain()).first;
    control.pid.SetDGain(param);

    param = controlSDF->Get("i_max", control.pid.IMax()).first;
    control.pid.SetIMax(param);

    param = controlSDF->Get("i_min", control.pid.IMin()).first;
    control.pid.SetIMin(param);

    param = controlSDF->Get("cmd_max", control.pid.CmdMax()).first;
    control.pid.SetCmdMax(param);

    param = controlSDF->Get("cmd_min", control.pid.CmdMin()).first;
    control.pid.SetCmdMin(param);

    // set pid initial command
    control.pid.SetCmd(0.0);

    this->dataPtr->controls.push_back(control);
    controlSDF = controlSDF->GetNextElement("control");
  }


  /*
  this->dataPtr->imuSensor = std::dynamic_pointer_cast<sensors::ImuSensor>
      (sensors::SensorManager::Instance()->GetSensor(imuScopedName[0]));

  if (!this->dataPtr->imuSensor)
  {
    if (imuScopedName.size() > 1)
    {
      ignwarn << "[" << this->dataPtr->modelName << "] "
             << "first imu_sensor scoped name [" << imuScopedName[0]
             << "] not found, trying the rest of the sensor names.\n";
      for (unsigned k = 1; k < imuScopedName.size(); ++k)
      {
        this->dataPtr->imuSensor = std::dynamic_pointer_cast<sensors::ImuSensor>
          (sensors::SensorManager::Instance()->GetSensor(imuScopedName[k]));
        if (this->dataPtr->imuSensor)
        {
          ignwarn << "found [" << imuScopedName[k] << "]\n";
          break;
        }
      }
    }

    if (!this->dataPtr->imuSensor)
    {
      ignwarn << "[" << this->dataPtr->modelName << "] "
             << "imu_sensor scoped name [" << imuName
             << "] not found, trying unscoped name.\n" << "\n";
      // TODO: this fails for multi-nested models.
      // TODO: and transforms fail for rotated nested model,
      //       joints point the wrong way.
      this->dataPtr->imuSensor = std::dynamic_pointer_cast<sensors::ImuSensor>
        (sensors::SensorManager::Instance()->GetSensor(imuName));
    }

    if (!this->dataPtr->imuSensor)
    {
      ignerr << "[" << this->dataPtr->modelName << "] "
            << "imu_sensor [" << imuName
            << "] not found, abort ArduPilot plugin.\n" << "\n";
      return;
    }
  }
  */
/* NOT MERGED IN MASTER YET
    // Get GPS
  std::string gpsName = _sdf->Get("imuName", static_cast<std::string>("gps_sensor")).first;
  std::vector<std::string> gpsScopedName = SensorScopedName(this->dataPtr->model, gpsName);
  if (gpsScopedName.size() > 1)
  {
    ignwarn << "[" << this->dataPtr->modelName << "] "
           << "multiple names match [" << gpsName << "] using first found"
           << " name.\n";
    for (unsigned k = 0; k < gpsScopedName.size(); ++k)
    {
      ignwarn << "  sensor " << k << " [" << gpsScopedName[k] << "].\n";
    }
  }

  if (gpsScopedName.size() > 0)
  {
    this->dataPtr->gpsSensor = std::dynamic_pointer_cast<sensors::GpsSensor>
      (sensors::SensorManager::Instance()->GetSensor(gpsScopedName[0]));
  }

  if (!this->dataPtr->gpsSensor)
  {
    if (gpsScopedName.size() > 1)
    {
      ignwarn << "[" << this->dataPtr->modelName << "] "
             << "first gps_sensor scoped name [" << gpsScopedName[0]
             << "] not found, trying the rest of the sensor names.\n";
      for (unsigned k = 1; k < gpsScopedName.size(); ++k)
      {
        this->dataPtr->gpsSensor = std::dynamic_pointer_cast<sensors::GpsSensor>
          (sensors::SensorManager::Instance()->GetSensor(gpsScopedName[k]));
        if (this->dataPtr->gpsSensor)
        {
          ignwarn << "found [" << gpsScopedName[k] << "]\n";
          break;
        }
      }
    }

    if (!this->dataPtr->gpsSensor)
    {
      ignwarn << "[" << this->dataPtr->modelName << "] "
             << "gps_sensor scoped name [" << gpsName
             << "] not found, trying unscoped name.\n" << "\n";
      this->dataPtr->gpsSensor = std::dynamic_pointer_cast<sensors::GpsSensor>
        (sensors::SensorManager::Instance()->GetSensor(gpsName));
    }
    
    if (!this->dataPtr->gpsSensor)
    {
      ignwarn << "[" << this->dataPtr->modelName << "] "
             << "gps [" << gpsName
             << "] not found, skipping gps support.\n" << "\n";
    }
    else
    {
      ignwarn << "[" << this->dataPtr->modelName << "] "
             << "  found "  << " [" << gpsName << "].\n";
    }
  }
  
  // Get Rangefinder
  // TODO add sonar
  std::string rangefinderName = _sdf->Get("rangefinderName",
    static_cast<std::string>("rangefinder_sensor")).first;
  std::vector<std::string> rangefinderScopedName = SensorScopedName(this->dataPtr->model, rangefinderName);
  if (rangefinderScopedName.size() > 1)
  {
    ignwarn << "[" << this->dataPtr->modelName << "] "
           << "multiple names match [" << rangefinderName << "] using first found"
           << " name.\n";
    for (unsigned k = 0; k < rangefinderScopedName.size(); ++k)
    {
      ignwarn << "  sensor " << k << " [" << rangefinderScopedName[k] << "].\n";
    }
  }

  if (rangefinderScopedName.size() > 0)
  {
    this->dataPtr->rangefinderSensor = std::dynamic_pointer_cast<sensors::RaySensor>
      (sensors::SensorManager::Instance()->GetSensor(rangefinderScopedName[0]));
  }

  if (!this->dataPtr->rangefinderSensor)
  {
    if (rangefinderScopedName.size() > 1)
    {
      ignwarn << "[" << this->dataPtr->modelName << "] "
             << "first rangefinder_sensor scoped name [" << rangefinderScopedName[0]
             << "] not found, trying the rest of the sensor names.\n";
      for (unsigned k = 1; k < rangefinderScopedName.size(); ++k)
      {
        this->dataPtr->rangefinderSensor = std::dynamic_pointer_cast<sensors::RaySensor>
          (sensors::SensorManager::Instance()->GetSensor(rangefinderScopedName[k]));
        if (this->dataPtr->rangefinderSensor)
        {
          ignwarn << "found [" << rangefinderScopedName[k] << "]\n";
          break;
        }
      }
    }

    if (!this->dataPtr->rangefinderSensor)
    {
      ignwarn << "[" << this->dataPtr->modelName << "] "
             << "rangefinder_sensor scoped name [" << rangefinderName
             << "] not found, trying unscoped name.\n" << "\n";
      /// TODO: this fails for multi-nested models.
      /// TODO: and transforms fail for rotated nested model,
      ///       joints point the wrong way.
      this->dataPtr->rangefinderSensor = std::dynamic_pointer_cast<sensors::RaySensor>
        (sensors::SensorManager::Instance()->GetSensor(rangefinderName));
    }
    if (!this->dataPtr->rangefinderSensor)
    {
      ignwarn << "[" << this->dataPtr->modelName << "] "
             << "ranfinder [" << rangefinderName
             << "] not found, skipping rangefinder support.\n" << "\n";
    }
    else
    {
      ignwarn << "[" << this->dataPtr->modelName << "] "
             << "  found "  << " [" << rangefinderName << "].\n";
    }
  }
*/

  // Initialise ardupilot sockets
  if (!InitArduPilotSockets(sdfClone))
  {
    return;
  }

  // Missed update count before we declare arduPilotOnline status false
  this->dataPtr->connectionTimeoutMaxCount =
    sdfClone->Get("connectionTimeoutMaxCount", 10).first;

  // Listen to the update event. This event is broadcast every simulation
  // iteration.
  //this->dataPtr->updateConnection = event::Events::ConnectWorldUpdateBegin(
      //std::bind(&ArduPilotPlugin::OnUpdate, this));

  ignlog << "[" << this->dataPtr->modelName << "] "
        << "ArduPilot ready to fly. The force will be with you" << std::endl;
}

/////////////////////////////////////////////////
void ignition::gazebo::systems::ArduPilotPlugin::PreUpdate(const ignition::gazebo::UpdateInfo &_info,
                                                           ignition::gazebo::EntityComponentManager &_ecm)
{
  // This lookup is done in PreUpdate() because in Configure() it's not possible to get the fully qualified topic name we want
  if(!this->dataPtr->imuInitialized)
  {
    // Set unconditionally because we're only going to try this once.
    this->dataPtr->imuInitialized = true;
    // TODO: use result from SDF (need to store from inside Configure())
    std::string imuName("imu_sensor");
    _ecm.Each<ignition::gazebo::components::Imu, ignition::gazebo::components::Name>(
            [&](const ignition::gazebo::Entity &_imu_entity,
                const ignition::gazebo::components::Imu * /*_imu*/,
                const ignition::gazebo::components::Name *_name)->bool
        {
          if(_name->Data() == imuName)
          {
            // The parent of the imu is imu_link
            ignition::gazebo::Entity parent = _ecm.ParentEntity(_imu_entity);
            if(parent != ignition::gazebo::kNullEntity)
            {
              // The grandparent of the imu is the quad itself, which is where this plugin is attached
              ignition::gazebo::Entity gparent = _ecm.ParentEntity(parent);
              if(gparent != ignition::gazebo::kNullEntity)
              {
                ignition::gazebo::Model gparent_model(gparent);
                ignerr << gparent_model.Name(_ecm) << " == " << this->dataPtr->modelName << std::endl;
                if(gparent_model.Name(_ecm) == this->dataPtr->modelName)
                {
                  this->dataPtr->imuTopicName = ignition::gazebo::scopedName(_imu_entity, _ecm) + "/imu";
                  igndbg << "Found IMU topic: " << this->dataPtr->imuTopicName << std::endl;
                }
              }
            }
          }
          return true;
        });
    
    if(this->dataPtr->imuTopicName.empty())
    {
      ignerr << "[" << this->dataPtr->modelName << "] "
            << "imu_sensor [" << imuName
            << "] not found, abort ArduPilot plugin." << "\n";
      return;
    }

    this->dataPtr->node.Subscribe(this->dataPtr->imuTopicName, &ignition::gazebo::systems::ArduPilotPluginPrivate::imuCb, this->dataPtr.get());
  }
}

/////////////////////////////////////////////////
void ignition::gazebo::systems::ArduPilotPlugin::PostUpdate(const ignition::gazebo::UpdateInfo &_info,
                                                            const ignition::gazebo::EntityComponentManager &_ecm)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);


  // Update the control surfaces and publish the new state.
  if (_info.simTime > this->dataPtr->lastControllerUpdateTime)
  {
    this->ReceiveMotorCommand();
    if (this->dataPtr->arduPilotOnline)
    {
      //this->ApplyMotorForces(std::chrono::duration_cast<std::chrono::duration<double> >(_info.simTime -
        //this->dataPtr->lastControllerUpdateTime).count(), _ecm);
      this->SendState(std::chrono::duration_cast<std::chrono::duration<double> >(_info.simTime).count(),
              _ecm);
    }
  }

  this->dataPtr->lastControllerUpdateTime = _info.simTime;
}

/////////////////////////////////////////////////
void ignition::gazebo::systems::ArduPilotPlugin::ResetPIDs()
{
  // Reset velocity PID for controls
  for (size_t i = 0; i < this->dataPtr->controls.size(); ++i)
  {
    this->dataPtr->controls[i].cmd = 0;
    // this->dataPtr->controls[i].pid.Reset();
  }
}

/////////////////////////////////////////////////
bool ignition::gazebo::systems::ArduPilotPlugin::InitArduPilotSockets(const std::shared_ptr<const sdf::Element> &_sdf) const
{
  this->dataPtr->fdm_addr =
    _sdf->Get("fdm_addr", static_cast<std::string>("127.0.0.1")).first;
  this->dataPtr->listen_addr =
    _sdf->Get("listen_addr", static_cast<std::string>("127.0.0.1")).first;
  this->dataPtr->fdm_port_in = 9002;
  this->dataPtr->fdm_port_in =
    static_cast<uint16_t>(_sdf->Get("fdm_port_in", static_cast<uint64_t>(9002)).first);
  this->dataPtr->fdm_port_out =
    static_cast<uint16_t>(_sdf->Get("fdm_port_out", static_cast<uint64_t>(9003)).first);

  if (!this->dataPtr->socket_in.Bind(this->dataPtr->listen_addr.c_str(),
      this->dataPtr->fdm_port_in))
  {
    ignerr << "[" << this->dataPtr->modelName << "] "
          << "failed to bind with " << this->dataPtr->listen_addr
          << ":" << this->dataPtr->fdm_port_in << " aborting plugin.\n";
    return false;
  }

  if (!this->dataPtr->socket_out.Connect(this->dataPtr->fdm_addr.c_str(),
      this->dataPtr->fdm_port_out))
  {
    ignerr << "[" << this->dataPtr->modelName << "] "
          << "failed to bind with " << this->dataPtr->fdm_addr
          << ":" << this->dataPtr->fdm_port_out << " aborting plugin.\n";
    return false;
  }

  return true;
}

/////////////////////////////////////////////////
#if 0
void ignition::gazebo::systems::ArduPilotPlugin::ApplyMotorForces(
        const double _dt,
        const ignition::gazebo::EntityComponentManager &_ecm)
{
  // update velocity PID for controls and apply force to joint
  for (size_t i = 0; i < this->dataPtr->controls.size(); ++i)
  {
    
    ignition::gazebo::components::JointForceCmd* jfc_comp = nullptr;
    ignition::gazebo::components::JointVelocityCmd* jvc_comp = nullptr;
    if (this->dataPtr->controls[i].useForce || this->dataPtr->controls[i].type == "EFFORT")
    {
      jfc_comp = _ecm.Component<ignition::gazebo::components::JointForceCmd>(this->dataPtr->controls[i].joint);
      if (jfc_comp == nullptr)
      {
        jfc_comp = _ecm.Component<ignition::gazebo::components::JointForceCmd>(
            _ecm.CreateComponent(this->dataPtr->controls[i].joint,
                                             ignition::gazebo::components::JointForceCmd({0})));
      }
    }
    else if (this->dataPtr->controls[i].type == "VELOCITY")
    {
      jvc_comp = _ecm.Component<ignition::gazebo::components::JointVelocityCmd>(this->dataPtr->controls[i].joint);
      if (jvc_comp == nullptr)
      {
        jvc_comp = _ecm.Component<ignition::gazebo::components::JointVelocityCmd>(
            _ecm.CreateComponent(this->dataPtr->controls[i].joint,
                             ignition::gazebo::components::JointVelocityCmd({0})));
      }
    }

    if (this->dataPtr->controls[i].useForce)
    {
      if (this->dataPtr->controls[i].type == "VELOCITY")
      {
        const double velTarget = this->dataPtr->controls[i].cmd /
          this->dataPtr->controls[i].rotorVelocitySlowdownSim;
        //const double vel = this->dataPtr->controls[i].joint->GetVelocity(0);
        auto v_comp = _ecm.Component<ignition::gazebo::components::JointVelocity>(this->dataPtr->controls[i].joint);
    if (v_comp->Data().empty())
    {
          igndbg << "[" << this->dataPtr->modelName << "] "
              << "No JointVelocity data available for control [" << i << "]. Skipping this control." << std::endl;
      continue;
    }
        const double vel = v_comp->Data()[0];
        const double error = vel - velTarget;
        const double force = this->dataPtr->controls[i].pid.Update(error, std::chrono::duration<double>(_dt));
        //this->dataPtr->controls[i].joint->SetForce(0, force);
    jfc_comp->Data()[0] = force;
      }
      else if (this->dataPtr->controls[i].type == "POSITION")
      {
        const double posTarget = this->dataPtr->controls[i].cmd;
        //const double pos = this->dataPtr->controls[i].joint->Position();
        auto p_comp = _ecm.Component<ignition::gazebo::components::JointPosition>(this->dataPtr->controls[i].joint);
        const double pos = p_comp->Data()[0];
        const double error = pos - posTarget;
        const double force = this->dataPtr->controls[i].pid.Update(error, std::chrono::duration<double>(_dt));
        //this->dataPtr->controls[i].joint->SetForce(0, force);
    jfc_comp->Data()[0] = force;
      }
      else if (this->dataPtr->controls[i].type == "EFFORT")
      {
        const double force = this->dataPtr->controls[i].cmd;
        //this->dataPtr->controls[i].joint->SetForce(0, force);
    jfc_comp->Data()[0] = force;
      }
      else
      {
        // do nothing
      }
    }
    else 
    {
      if (this->dataPtr->controls[i].type == "VELOCITY")
      {
        //this->dataPtr->controls[i].joint->SetVelocity(0, this->dataPtr->controls[i].cmd);
    jvc_comp->Data()[0] = this->dataPtr->controls[i].cmd;
      }
      else if (this->dataPtr->controls[i].type == "POSITION")
      {
        //this->dataPtr->controls[i].joint->SetPosition(0, this->dataPtr->controls[i].cmd);
    //TODO: figure out whether position control matters, and if so, how to use it.
    ignwarn << "Failed to do position control on joint " << i << 
        " because there's no JointPositionCmd component (yet?)" << std::endl;
      }
      else if (this->dataPtr->controls[i].type == "EFFORT")
      {
        const double force = this->dataPtr->controls[i].cmd;
        //this->dataPtr->controls[i].joint->SetForce(0, force);
    jfc_comp->Data()[0] = force;
      }
      else
      {
        // do nothing
      }
    }
  }
}

#endif

/////////////////////////////////////////////////
void ignition::gazebo::systems::ArduPilotPlugin::ReceiveMotorCommand()
{
  // Added detection for whether ArduPilot is online or not.
  // If ArduPilot is detected (receive of fdm packet from someone),
  // then socket receive wait time is increased from 1ms to 1 sec
  // to accomodate network jitter.
  // If ArduPilot is not detected, receive call blocks for 1ms
  // on each call.
  // Once ArduPilot presence is detected, it takes this many
  // missed receives before declaring the FCS offline.

  ServoPacket pkt;
  uint32_t waitMs;
  if (this->dataPtr->arduPilotOnline)
  {
    // increase timeout for receive once we detect a packet from
    // ArduPilot FCS.
    waitMs = 1000;
  }
  else
  {
    // Otherwise skip quickly and do not set control force.
    waitMs = 1;
  }
  ssize_t recvSize =
    this->dataPtr->socket_in.Recv(&pkt, sizeof(ServoPacket), waitMs);

  // Drain the socket in the case we're backed up
  int counter = 0;
  ServoPacket last_pkt;
  while (true)
  {
    // last_pkt = pkt;
    const ssize_t recvSize_last =
      this->dataPtr->socket_in.Recv(&last_pkt, sizeof(ServoPacket), 0ul);
    if (recvSize_last == -1)
    {
      break;
    }
    counter++;
    pkt = last_pkt;
    recvSize = recvSize_last;
  }
  if (counter > 0)
  {
    igndbg << "[" << this->dataPtr->modelName << "] "
          << "Drained n packets: " << counter << std::endl;
  }

  if (recvSize == -1)
  {
    // didn't receive a packet
    // igndbg << "no packet\n";
    ignition::common::Time::Sleep(ignition::common::Time(0,100));
    if (this->dataPtr->arduPilotOnline)
    {
      ignwarn << "[" << this->dataPtr->modelName << "] "
             << "Broken ArduPilot connection, count ["
             << this->dataPtr->connectionTimeoutCount
             << "/" << this->dataPtr->connectionTimeoutMaxCount
             << "]\n";
      if (++this->dataPtr->connectionTimeoutCount >
        this->dataPtr->connectionTimeoutMaxCount)
      {
        this->dataPtr->connectionTimeoutCount = 0;
        this->dataPtr->arduPilotOnline = false;
        ignwarn << "[" << this->dataPtr->modelName << "] "
               << "Broken ArduPilot connection, resetting motor control.\n";
        this->ResetPIDs();
      }
    }
  }
  else
  {
    const ssize_t expectedPktSize =
    sizeof(pkt.motorSpeed[0]) * this->dataPtr->controls.size();
    if (recvSize < expectedPktSize)
    {
      ignerr << "[" << this->dataPtr->modelName << "] "
            << "got less than model needs. Got: " << recvSize
            << "commands, expected size: " << expectedPktSize << "\n";
    }
    const ssize_t recvChannels = recvSize / sizeof(pkt.motorSpeed[0]);
    // for(unsigned int i = 0; i < recvChannels; ++i)
    // {
    //   igndbg << "servo_command [" << i << "]: " << pkt.motorSpeed[i] << "\n";
    // }

    if (!this->dataPtr->arduPilotOnline)
    {
      igndbg << "[" << this->dataPtr->modelName << "] "
            << "ArduPilot controller online detected.\n";
      // made connection, set some flags
      this->dataPtr->connectionTimeoutCount = 0;
      this->dataPtr->arduPilotOnline = true;
    }

    // compute command based on requested motorSpeed
    for (unsigned i = 0; i < this->dataPtr->controls.size(); ++i)
    {
      if (i < MAX_MOTORS)
      {
        if (this->dataPtr->controls[i].channel < recvChannels)
        {
          // bound incoming cmd between 0 and 1
          const double cmd = ignition::math::clamp(
            pkt.motorSpeed[this->dataPtr->controls[i].channel],
            -1.0f, 1.0f);
          this->dataPtr->controls[i].cmd =
            this->dataPtr->controls[i].multiplier *
            (this->dataPtr->controls[i].offset + cmd);
          // igndbg << "apply input chan[" << this->dataPtr->controls[i].channel
          //       << "] to control chan[" << i
          //       << "] with joint name ["
          //       << this->dataPtr->controls[i].jointName
          //       << "] raw cmd ["
          //       << pkt.motorSpeed[this->dataPtr->controls[i].channel]
          //       << "] adjusted cmd [" << this->dataPtr->controls[i].cmd
          //       << "].\n";
        }
        else
        {
          ignerr << "[" << this->dataPtr->modelName << "] "
                << "control[" << i << "] channel ["
                << this->dataPtr->controls[i].channel
                << "] is greater than incoming commands size["
                << recvChannels
                << "], control not applied.\n";
        }
      }
      else
      {
        ignerr << "[" << this->dataPtr->modelName << "] "
              << "too many motors, skipping [" << i
              << " > " << MAX_MOTORS << "].\n";
      }
    }
  }
}

/////////////////////////////////////////////////
void ignition::gazebo::systems::ArduPilotPlugin::SendState(double _simTime,
      const ignition::gazebo::EntityComponentManager &_ecm) const
{
#if 0
  // send_fdm
  fdmPacket pkt;

  pkt.timestamp = _simTime;
  //ignerr << "timestamp [" << pkt.timestamp << "]\n";

  // asssumed that the imu orientation is:
  //   x forward
  //   y right
  //   z down
  // get linear acceleration in body frame
  auto la_comp = _ecm.Component<ignition::gazebo::components::LinearAcceleration>(this->dataPtr->imu);
  const ignition::math::Vector3d linearAccel = la_comp->Data();
    //this->dataPtr->imuSensor->LinearAcceleration();

  // copy to pkt
  pkt.imuLinearAccelerationXYZ[0] = linearAccel.X();
  pkt.imuLinearAccelerationXYZ[1] = linearAccel.Y();
  pkt.imuLinearAccelerationXYZ[2] = linearAccel.Z();
  //ignerr << "lin accel [" << linearAccel << "]\n";

  // get angular velocity in body frame
  auto av_comp = _ecm.Component<ignition::gazebo::components::AngularVelocity>(this->dataPtr->imu);
  const ignition::math::Vector3d angularVel = av_comp->Data();
    //this->dataPtr->imuSensor->AngularVelocity();

  // copy to pkt
  pkt.imuAngularVelocityRPY[0] = angularVel.X();
  pkt.imuAngularVelocityRPY[1] = angularVel.Y();
  pkt.imuAngularVelocityRPY[2] = angularVel.Z();
  //ignerr << "ang vel [" << angularVel << "]\n";
 
  // get inertial pose and velocity
  // position of the uav in world frame
  // this position is used to calcualte bearing and distance
  // from starting location, then use that to update gps position.
  // The algorithm looks something like below (from ardupilot helper
  // libraries):
  //   bearing = to_degrees(atan2(position.y, position.x));
  //   distance = math.sqrt(self.position.x**2 + self.position.y**2)
  //   (self.latitude, self.longitude) = util.gps_newpos(
  //    self.home_latitude, self.home_longitude, bearing, distance)
  // where xyz is in the NED directions.
  // Gazebo world xyz is assumed to be N, -E, -D, so flip some stuff
  // around.
  // orientation of the uav in world NED frame -
  // assuming the world NED frame has xyz mapped to NED,
  // imuLink is NED - z down

  // model world pose brings us to model,
  // which for example zephyr has -y-forward, x-left, z-up
  // adding modelXYZToAirplaneXForwardZDown rotates
  //   from: model XYZ
  //   to: airplane x-forward, y-left, z-down
  auto p_comp = _ecm.Component<ignition::gazebo::components::Pose>(this->dataPtr->entity);
  const ignition::math::Pose3d gazeboXYZToModelXForwardZDown =
    this->modelXYZToAirplaneXForwardZDown +
    p_comp->Data();
    //this->dataPtr->model->WorldPose();

  // get transform from world NED to Model frame
  const ignition::math::Pose3d NEDToModelXForwardZUp =
    gazeboXYZToModelXForwardZDown - this->gazeboXYZToNED;

  // ignerr << "ned to model [" << NEDToModelXForwardZUp << "]\n";

  // N
  pkt.positionXYZ[0] = NEDToModelXForwardZUp.Pos().X();

  // E
  pkt.positionXYZ[1] = NEDToModelXForwardZUp.Pos().Y();

  // D
  pkt.positionXYZ[2] = NEDToModelXForwardZUp.Pos().Z();

  // imuOrientationQuat is the rotation from world NED frame
  // to the uav frame.
  pkt.imuOrientationQuat[0] = NEDToModelXForwardZUp.Rot().W();
  pkt.imuOrientationQuat[1] = NEDToModelXForwardZUp.Rot().X();
  pkt.imuOrientationQuat[2] = NEDToModelXForwardZUp.Rot().Y();
  pkt.imuOrientationQuat[3] = NEDToModelXForwardZUp.Rot().Z();
  //ignerr << "NEDToModelXForwardZUp [" << NEDToModelXForwardZUp << "]\n";

  // igndbg << "imu [" << gazeboXYZToModelXForwardZDown.rot.GetAsEuler()
  //       << "]\n";
  // igndbg << "ned [" << this->gazeboXYZToNED.rot.GetAsEuler() << "]\n";
  // igndbg << "rot [" << NEDToModelXForwardZUp.rot.GetAsEuler() << "]\n";

  // Get NED velocity in body frame *
  // or...
  // Get model velocity in NED frame
  /*
  auto v_comp = _ecm.Component<ignition::gazebo::components::LinearVelocity>(this->dataPtr->entity);
  if (v_comp == nullptr)
  {
    //v_comp = _ecm.Component<ignition::gazebo::components::LinearVelocity>(
            //_ecm.CreateComponent(this->dataPtr->entity,
                //ignition::gazebo::components::LinearVelocity()));
    igndbg << "[" << this->dataPtr->modelName << "] "
           << "No LinearVelocity data available. Not sending state to ArduPilot." << std::endl;
    return;
  }
  const ignition::math::Vector3d velGazeboWorldFrame = v_comp->Data();
  */
  const ignition::math::Vector3d velGazeboWorldFrame;

    //this->dataPtr->model->GetLink()->WorldLinearVel();
  const ignition::math::Vector3d velNEDFrame =
    this->gazeboXYZToNED.Rot().RotateVectorReverse(velGazeboWorldFrame);
  pkt.velocityXYZ[0] = velNEDFrame.X();
  pkt.velocityXYZ[1] = velNEDFrame.Y();
  pkt.velocityXYZ[2] = velNEDFrame.Z();
  //ignerr << "velNEDFrame [" << velNEDFrame << "]\n";
/* NOT MERGED IN MASTER YET
  if (!this->dataPtr->gpsSensor)
    {

    }
    else {
        pkt.longitude = this->dataPtr->gpsSensor->Longitude().Degree();
        pkt.latitude = this->dataPtr->gpsSensor->Latitude().Degree();
        pkt.altitude = this->dataPtr->gpsSensor->Altitude();
    }
    
    // TODO : make generic enough to accept sonar/gpuray etc. too
    if (!this->dataPtr->rangefinderSensor)
    {

    } else {
        // Rangefinder value can not be send as Inf to ardupilot
        const double range = this->dataPtr->rangefinderSensor->Range(0);
        pkt.rangefinder = std::isinf(range) ? 0.0 : range;
    }
    
  // airspeed :     wind = Vector3(environment.wind.x, environment.wind.y, environment.wind.z)
   // pkt.airspeed = (pkt.velocity - wind).length()
*/
  printf("Sending fdmPacket:\n"
          "%f [%f %f %f] [%f %f %f] [%f %f %f %f] [%f %f %f] [%f %f %f]\n",
          pkt.timestamp,
          pkt.imuAngularVelocityRPY[0], pkt.imuAngularVelocityRPY[1], pkt.imuAngularVelocityRPY[2],
          pkt.imuLinearAccelerationXYZ[0], pkt.imuLinearAccelerationXYZ[1], pkt.imuLinearAccelerationXYZ[2],
          pkt.imuOrientationQuat[0], pkt.imuOrientationQuat[1], pkt.imuOrientationQuat[2], pkt.imuOrientationQuat[3],
          pkt.velocityXYZ[0], pkt.velocityXYZ[1], pkt.velocityXYZ[2],
          pkt.positionXYZ[0], pkt.positionXYZ[1], pkt.positionXYZ[2]);
  this->dataPtr->socket_out.Send(&pkt, sizeof(pkt));
#endif
}

// Register plugin
IGNITION_ADD_PLUGIN(ignition::gazebo::systems::ArduPilotPlugin,
                    ignition::gazebo::System,
                    ignition::gazebo::systems::ArduPilotPlugin::ISystemConfigure,
                    ignition::gazebo::systems::ArduPilotPlugin::ISystemPostUpdate,
                    ignition::gazebo::systems::ArduPilotPlugin::ISystemPreUpdate)
// Add plugin alias so that we can refer to the plugin without the version
// namespace
IGNITION_ADD_PLUGIN_ALIAS(ignition::gazebo::systems::ArduPilotPlugin,"ArduPilotPlugin")
