;; Auto-generated. Do not edit!


(when (boundp 'lidar::point_msg)
  (if (not (find-package "LIDAR"))
    (make-package "LIDAR"))
  (shadow 'point_msg (find-package "LIDAR")))
(unless (find-package "LIDAR::POINT_MSG")
  (make-package "LIDAR::POINT_MSG"))

(in-package "ROS")
;;//! \htmlinclude point_msg.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass lidar::point_msg
  :super ros::object
  :slots (_points _width _height ))

(defmethod lidar::point_msg
  (:init
   (&key
    ((:points __points) (let (r) (dotimes (i 0) (push (instance geometry_msgs::Vector3 :init) r)) r))
    ((:width __width) 0)
    ((:height __height) 0)
    )
   (send-super :init)
   (setq _points __points)
   (setq _width (round __width))
   (setq _height (round __height))
   self)
  (:points
   (&rest __points)
   (if (keywordp (car __points))
       (send* _points __points)
     (progn
       (if __points (setq _points (car __points)))
       _points)))
  (:width
   (&optional __width)
   (if __width (setq _width __width)) _width)
  (:height
   (&optional __height)
   (if __height (setq _height __height)) _height)
  (:serialization-length
   ()
   (+
    ;; geometry_msgs/Vector3[] _points
    (apply #'+ (send-all _points :serialization-length)) 4
    ;; uint16 _width
    2
    ;; uint16 _height
    2
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; geometry_msgs/Vector3[] _points
     (write-long (length _points) s)
     (dolist (elem _points)
       (send elem :serialize s)
       )
     ;; uint16 _width
       (write-word _width s)
     ;; uint16 _height
       (write-word _height s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; geometry_msgs/Vector3[] _points
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _points (let (r) (dotimes (i n) (push (instance geometry_msgs::Vector3 :init) r)) r))
     (dolist (elem- _points)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; uint16 _width
     (setq _width (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint16 _height
     (setq _height (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;;
   self)
  )

(setf (get lidar::point_msg :md5sum-) "4488507b5dfbcd949fac00cea72e5149")
(setf (get lidar::point_msg :datatype-) "lidar/point_msg")
(setf (get lidar::point_msg :definition-)
      "geometry_msgs/Vector3[] points
uint16 width
uint16 height

================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
float64 z
")



(provide :lidar/point_msg "4488507b5dfbcd949fac00cea72e5149")


