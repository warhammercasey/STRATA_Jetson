; Auto-generated. Do not edit!


(cl:in-package lidar-msg)


;//! \htmlinclude point_msg.msg.html

(cl:defclass <point_msg> (roslisp-msg-protocol:ros-message)
  ((points
    :reader points
    :initarg :points
    :type (cl:vector geometry_msgs-msg:Vector3)
   :initform (cl:make-array 0 :element-type 'geometry_msgs-msg:Vector3 :initial-element (cl:make-instance 'geometry_msgs-msg:Vector3)))
   (width
    :reader width
    :initarg :width
    :type cl:fixnum
    :initform 0)
   (height
    :reader height
    :initarg :height
    :type cl:fixnum
    :initform 0))
)

(cl:defclass point_msg (<point_msg>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <point_msg>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'point_msg)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name lidar-msg:<point_msg> is deprecated: use lidar-msg:point_msg instead.")))

(cl:ensure-generic-function 'points-val :lambda-list '(m))
(cl:defmethod points-val ((m <point_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader lidar-msg:points-val is deprecated.  Use lidar-msg:points instead.")
  (points m))

(cl:ensure-generic-function 'width-val :lambda-list '(m))
(cl:defmethod width-val ((m <point_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader lidar-msg:width-val is deprecated.  Use lidar-msg:width instead.")
  (width m))

(cl:ensure-generic-function 'height-val :lambda-list '(m))
(cl:defmethod height-val ((m <point_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader lidar-msg:height-val is deprecated.  Use lidar-msg:height instead.")
  (height m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <point_msg>) ostream)
  "Serializes a message object of type '<point_msg>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'points))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'points))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'width)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'width)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'height)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'height)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <point_msg>) istream)
  "Deserializes a message object of type '<point_msg>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'points) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'points)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'geometry_msgs-msg:Vector3))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'width)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'width)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'height)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'height)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<point_msg>)))
  "Returns string type for a message object of type '<point_msg>"
  "lidar/point_msg")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'point_msg)))
  "Returns string type for a message object of type 'point_msg"
  "lidar/point_msg")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<point_msg>)))
  "Returns md5sum for a message object of type '<point_msg>"
  "4488507b5dfbcd949fac00cea72e5149")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'point_msg)))
  "Returns md5sum for a message object of type 'point_msg"
  "4488507b5dfbcd949fac00cea72e5149")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<point_msg>)))
  "Returns full string definition for message of type '<point_msg>"
  (cl:format cl:nil "geometry_msgs/Vector3[] points~%uint16 width~%uint16 height~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'point_msg)))
  "Returns full string definition for message of type 'point_msg"
  (cl:format cl:nil "geometry_msgs/Vector3[] points~%uint16 width~%uint16 height~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <point_msg>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'points) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     2
     2
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <point_msg>))
  "Converts a ROS message object to a list"
  (cl:list 'point_msg
    (cl:cons ':points (points msg))
    (cl:cons ':width (width msg))
    (cl:cons ':height (height msg))
))
