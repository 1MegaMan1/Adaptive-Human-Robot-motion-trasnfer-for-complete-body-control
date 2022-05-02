
(cl:in-package :asdf)

(defsystem "kinect_v2-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
)
  :components ((:file "_package")
    (:file "BodyJoints" :depends-on ("_package_BodyJoints"))
    (:file "_package_BodyJoints" :depends-on ("_package"))
  ))