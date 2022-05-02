;; Auto-generated. Do not edit!


(when (boundp 'kinect_v2::BodyJoints)
  (if (not (find-package "KINECT_V2"))
    (make-package "KINECT_V2"))
  (shadow 'BodyJoints (find-package "KINECT_V2")))
(unless (find-package "KINECT_V2::BODYJOINTS")
  (make-package "KINECT_V2::BODYJOINTS"))

(in-package "ROS")
;;//! \htmlinclude BodyJoints.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass kinect_v2::BodyJoints
  :super ros::object
  :slots (_user_id _tracked _joints ))

(defmethod kinect_v2::BodyJoints
  (:init
   (&key
    ((:user_id __user_id) 0)
    ((:tracked __tracked) "")
    ((:joints __joints) (let (r) (dotimes (i 16) (push (instance geometry_msgs::Pose :init) r)) r))
    )
   (send-super :init)
   (setq _user_id (round __user_id))
   (setq _tracked (string __tracked))
   (setq _joints __joints)
   self)
  (:user_id
   (&optional __user_id)
   (if __user_id (setq _user_id __user_id)) _user_id)
  (:tracked
   (&optional __tracked)
   (if __tracked (setq _tracked __tracked)) _tracked)
  (:joints
   (&rest __joints)
   (if (keywordp (car __joints))
       (send* _joints __joints)
     (progn
       (if __joints (setq _joints (car __joints)))
       _joints)))
  (:serialization-length
   ()
   (+
    ;; int32 _user_id
    4
    ;; string _tracked
    4 (length _tracked)
    ;; geometry_msgs/Pose[16] _joints
    (apply #'+ (send-all _joints :serialization-length))
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _user_id
       (write-long _user_id s)
     ;; string _tracked
       (write-long (length _tracked) s) (princ _tracked s)
     ;; geometry_msgs/Pose[16] _joints
     (dolist (elem _joints)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _user_id
     (setq _user_id (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; string _tracked
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _tracked (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; geometry_msgs/Pose[16] _joints
   (dotimes (i 16)
     (send (elt _joints i) :deserialize buf ptr-) (incf ptr- (send (elt _joints i) :serialization-length))
     )
   ;;
   self)
  )

(setf (get kinect_v2::BodyJoints :md5sum-) "61535990ee807ee844649627b51297c2")
(setf (get kinect_v2::BodyJoints :datatype-) "kinect_v2/BodyJoints")
(setf (get kinect_v2::BodyJoints :definition-)
      "int32 user_id
string tracked
geometry_msgs/Pose[16] joints

================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w

")



(provide :kinect_v2/BodyJoints "61535990ee807ee844649627b51297c2")


