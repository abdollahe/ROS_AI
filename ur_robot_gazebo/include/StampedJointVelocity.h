// Generated by gencpp from file ur_robot_gazebo/StampedJointVelocity.msg
// DO NOT EDIT!


#ifndef UR_ROBOT_GAZEBO_MESSAGE_STAMPEDJOINTVELOCITY_H
#define UR_ROBOT_GAZEBO_MESSAGE_STAMPEDJOINTVELOCITY_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>

namespace ur_robot_gazebo
{
template <class ContainerAllocator>
struct StampedJointVelocity_
{
  typedef StampedJointVelocity_<ContainerAllocator> Type;

  StampedJointVelocity_()
    : header()
    , velocity()
    , jointName()  {
    }
  StampedJointVelocity_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , velocity(_alloc)
    , jointName(_alloc)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef std::vector<double, typename ContainerAllocator::template rebind<double>::other >  _velocity_type;
  _velocity_type velocity;

   typedef std::vector<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > , typename ContainerAllocator::template rebind<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::other >  _jointName_type;
  _jointName_type jointName;





  typedef boost::shared_ptr< ::ur_robot_gazebo::StampedJointVelocity_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::ur_robot_gazebo::StampedJointVelocity_<ContainerAllocator> const> ConstPtr;

}; // struct StampedJointVelocity_

typedef ::ur_robot_gazebo::StampedJointVelocity_<std::allocator<void> > StampedJointVelocity;

typedef boost::shared_ptr< ::ur_robot_gazebo::StampedJointVelocity > StampedJointVelocityPtr;
typedef boost::shared_ptr< ::ur_robot_gazebo::StampedJointVelocity const> StampedJointVelocityConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::ur_robot_gazebo::StampedJointVelocity_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::ur_robot_gazebo::StampedJointVelocity_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace ur_robot_gazebo

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': True}
// {'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'ur_robot_gazebo': ['/home/abdollah/catkin_ws/src/skill_transfer/ur_robot_gazebo/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::ur_robot_gazebo::StampedJointVelocity_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::ur_robot_gazebo::StampedJointVelocity_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ur_robot_gazebo::StampedJointVelocity_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ur_robot_gazebo::StampedJointVelocity_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ur_robot_gazebo::StampedJointVelocity_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ur_robot_gazebo::StampedJointVelocity_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::ur_robot_gazebo::StampedJointVelocity_<ContainerAllocator> >
{
  static const char* value()
  {
    return "19d49d195cda4965c1ba548368d2bfb2";
  }

  static const char* value(const ::ur_robot_gazebo::StampedJointVelocity_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x19d49d195cda4965ULL;
  static const uint64_t static_value2 = 0xc1ba548368d2bfb2ULL;
};

template<class ContainerAllocator>
struct DataType< ::ur_robot_gazebo::StampedJointVelocity_<ContainerAllocator> >
{
  static const char* value()
  {
    return "ur_robot_gazebo/StampedJointVelocity";
  }

  static const char* value(const ::ur_robot_gazebo::StampedJointVelocity_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::ur_robot_gazebo::StampedJointVelocity_<ContainerAllocator> >
{
  static const char* value()
  {
    return "std_msgs/Header header\n\
float64[] velocity\n\
string[] jointName\n\
================================================================================\n\
MSG: std_msgs/Header\n\
# Standard metadata for higher-level stamped data types.\n\
# This is generally used to communicate timestamped data \n\
# in a particular coordinate frame.\n\
# \n\
# sequence ID: consecutively increasing ID \n\
uint32 seq\n\
#Two-integer timestamp that is expressed as:\n\
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n\
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n\
# time-handling sugar is provided by the client library\n\
time stamp\n\
#Frame this data is associated with\n\
# 0: no frame\n\
# 1: global frame\n\
string frame_id\n\
";
  }

  static const char* value(const ::ur_robot_gazebo::StampedJointVelocity_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::ur_robot_gazebo::StampedJointVelocity_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.velocity);
      stream.next(m.jointName);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct StampedJointVelocity_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::ur_robot_gazebo::StampedJointVelocity_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::ur_robot_gazebo::StampedJointVelocity_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "velocity[]" << std::endl;
    for (size_t i = 0; i < v.velocity.size(); ++i)
    {
      s << indent << "  velocity[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.velocity[i]);
    }
    s << indent << "jointName[]" << std::endl;
    for (size_t i = 0; i < v.jointName.size(); ++i)
    {
      s << indent << "  jointName[" << i << "]: ";
      Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.jointName[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // UR_ROBOT_GAZEBO_MESSAGE_STAMPEDJOINTVELOCITY_H
