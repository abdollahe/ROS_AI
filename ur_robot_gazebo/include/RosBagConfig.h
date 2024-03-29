// Generated by gencpp from file ur_robot_gazebo/RosBagConfig.msg
// DO NOT EDIT!


#ifndef UR_ROBOT_GAZEBO_MESSAGE_ROSBAGCONFIG_H
#define UR_ROBOT_GAZEBO_MESSAGE_ROSBAGCONFIG_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Time.h>

namespace ur_robot_gazebo
{
template <class ContainerAllocator>
struct RosBagConfig_
{
  typedef RosBagConfig_<ContainerAllocator> Type;

  RosBagConfig_()
    : time()
    , topic_name()  {
    }
  RosBagConfig_(const ContainerAllocator& _alloc)
    : time(_alloc)
    , topic_name(_alloc)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Time_<ContainerAllocator>  _time_type;
  _time_type time;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _topic_name_type;
  _topic_name_type topic_name;





  typedef boost::shared_ptr< ::ur_robot_gazebo::RosBagConfig_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::ur_robot_gazebo::RosBagConfig_<ContainerAllocator> const> ConstPtr;

}; // struct RosBagConfig_

typedef ::ur_robot_gazebo::RosBagConfig_<std::allocator<void> > RosBagConfig;

typedef boost::shared_ptr< ::ur_robot_gazebo::RosBagConfig > RosBagConfigPtr;
typedef boost::shared_ptr< ::ur_robot_gazebo::RosBagConfig const> RosBagConfigConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::ur_robot_gazebo::RosBagConfig_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::ur_robot_gazebo::RosBagConfig_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace ur_robot_gazebo

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'ur_robot_gazebo': ['/home/abdollah/catkin_ws/src/skill_transfer/ur_robot_gazebo/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::ur_robot_gazebo::RosBagConfig_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::ur_robot_gazebo::RosBagConfig_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ur_robot_gazebo::RosBagConfig_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ur_robot_gazebo::RosBagConfig_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ur_robot_gazebo::RosBagConfig_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ur_robot_gazebo::RosBagConfig_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::ur_robot_gazebo::RosBagConfig_<ContainerAllocator> >
{
  static const char* value()
  {
    return "cc11b00a2af1a84550115d7bd8d331db";
  }

  static const char* value(const ::ur_robot_gazebo::RosBagConfig_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xcc11b00a2af1a845ULL;
  static const uint64_t static_value2 = 0x50115d7bd8d331dbULL;
};

template<class ContainerAllocator>
struct DataType< ::ur_robot_gazebo::RosBagConfig_<ContainerAllocator> >
{
  static const char* value()
  {
    return "ur_robot_gazebo/RosBagConfig";
  }

  static const char* value(const ::ur_robot_gazebo::RosBagConfig_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::ur_robot_gazebo::RosBagConfig_<ContainerAllocator> >
{
  static const char* value()
  {
    return "std_msgs/Time time\n\
string topic_name\n\
================================================================================\n\
MSG: std_msgs/Time\n\
time data\n\
";
  }

  static const char* value(const ::ur_robot_gazebo::RosBagConfig_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::ur_robot_gazebo::RosBagConfig_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.time);
      stream.next(m.topic_name);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct RosBagConfig_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::ur_robot_gazebo::RosBagConfig_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::ur_robot_gazebo::RosBagConfig_<ContainerAllocator>& v)
  {
    s << indent << "time: ";
    s << std::endl;
    Printer< ::std_msgs::Time_<ContainerAllocator> >::stream(s, indent + "  ", v.time);
    s << indent << "topic_name: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.topic_name);
  }
};

} // namespace message_operations
} // namespace ros

#endif // UR_ROBOT_GAZEBO_MESSAGE_ROSBAGCONFIG_H
