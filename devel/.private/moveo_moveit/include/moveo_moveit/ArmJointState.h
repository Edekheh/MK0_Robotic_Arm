// Generated by gencpp from file moveo_moveit/ArmJointState.msg
// DO NOT EDIT!


#ifndef MOVEO_MOVEIT_MESSAGE_ARMJOINTSTATE_H
#define MOVEO_MOVEIT_MESSAGE_ARMJOINTSTATE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace moveo_moveit
{
template <class ContainerAllocator>
struct ArmJointState_
{
  typedef ArmJointState_<ContainerAllocator> Type;

  ArmJointState_()
    : position1(0)
    , position2(0)
    , position3(0)
    , position4(0)
    , position5(0)
    , position6(0)  {
    }
  ArmJointState_(const ContainerAllocator& _alloc)
    : position1(0)
    , position2(0)
    , position3(0)
    , position4(0)
    , position5(0)
    , position6(0)  {
  (void)_alloc;
    }



   typedef int16_t _position1_type;
  _position1_type position1;

   typedef int16_t _position2_type;
  _position2_type position2;

   typedef int16_t _position3_type;
  _position3_type position3;

   typedef int16_t _position4_type;
  _position4_type position4;

   typedef int16_t _position5_type;
  _position5_type position5;

   typedef int16_t _position6_type;
  _position6_type position6;





  typedef boost::shared_ptr< ::moveo_moveit::ArmJointState_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::moveo_moveit::ArmJointState_<ContainerAllocator> const> ConstPtr;

}; // struct ArmJointState_

typedef ::moveo_moveit::ArmJointState_<std::allocator<void> > ArmJointState;

typedef boost::shared_ptr< ::moveo_moveit::ArmJointState > ArmJointStatePtr;
typedef boost::shared_ptr< ::moveo_moveit::ArmJointState const> ArmJointStateConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::moveo_moveit::ArmJointState_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::moveo_moveit::ArmJointState_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace moveo_moveit

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'moveo_moveit': ['/home/fshodzoncy/robotic_arm/src/moveo_ros/moveo_moveit/msg'], 'sensor_msgs': ['/opt/ros/kinetic/share/sensor_msgs/cmake/../msg'], 'geometry_msgs': ['/opt/ros/kinetic/share/geometry_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::moveo_moveit::ArmJointState_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::moveo_moveit::ArmJointState_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::moveo_moveit::ArmJointState_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::moveo_moveit::ArmJointState_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::moveo_moveit::ArmJointState_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::moveo_moveit::ArmJointState_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::moveo_moveit::ArmJointState_<ContainerAllocator> >
{
  static const char* value()
  {
    return "7bf56d1cde4c613af8c16b02c640040e";
  }

  static const char* value(const ::moveo_moveit::ArmJointState_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x7bf56d1cde4c613aULL;
  static const uint64_t static_value2 = 0xf8c16b02c640040eULL;
};

template<class ContainerAllocator>
struct DataType< ::moveo_moveit::ArmJointState_<ContainerAllocator> >
{
  static const char* value()
  {
    return "moveo_moveit/ArmJointState";
  }

  static const char* value(const ::moveo_moveit::ArmJointState_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::moveo_moveit::ArmJointState_<ContainerAllocator> >
{
  static const char* value()
  {
    return "int16 position1\n\
int16 position2\n\
int16 position3\n\
int16 position4\n\
int16 position5\n\
int16 position6\n\
";
  }

  static const char* value(const ::moveo_moveit::ArmJointState_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::moveo_moveit::ArmJointState_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.position1);
      stream.next(m.position2);
      stream.next(m.position3);
      stream.next(m.position4);
      stream.next(m.position5);
      stream.next(m.position6);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct ArmJointState_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::moveo_moveit::ArmJointState_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::moveo_moveit::ArmJointState_<ContainerAllocator>& v)
  {
    s << indent << "position1: ";
    Printer<int16_t>::stream(s, indent + "  ", v.position1);
    s << indent << "position2: ";
    Printer<int16_t>::stream(s, indent + "  ", v.position2);
    s << indent << "position3: ";
    Printer<int16_t>::stream(s, indent + "  ", v.position3);
    s << indent << "position4: ";
    Printer<int16_t>::stream(s, indent + "  ", v.position4);
    s << indent << "position5: ";
    Printer<int16_t>::stream(s, indent + "  ", v.position5);
    s << indent << "position6: ";
    Printer<int16_t>::stream(s, indent + "  ", v.position6);
  }
};

} // namespace message_operations
} // namespace ros

#endif // MOVEO_MOVEIT_MESSAGE_ARMJOINTSTATE_H
