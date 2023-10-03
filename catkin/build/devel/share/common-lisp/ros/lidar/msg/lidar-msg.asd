
(cl:in-package :asdf)

(defsystem "lidar-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
)
  :components ((:file "_package")
    (:file "point_msg" :depends-on ("_package_point_msg"))
    (:file "_package_point_msg" :depends-on ("_package"))
  ))