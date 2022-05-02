; Auto-generated. Do not edit!


(cl:in-package kinect_v2-msg)


;//! \htmlinclude BodyJoints.msg.html

(cl:defclass <BodyJoints> (roslisp-msg-protocol:ros-message)
  ((user_id
    :reader user_id
    :initarg :user_id
    :type cl:integer
    :initform 0)
   (tracked
    :reader tracked
    :initarg :tracked
    :type cl:string
    :initform "")
   (joints
    :reader joints
    :initarg :joints
    :type (cl:vector geometry_msgs-msg:Pose)
   :initform (cl:make-array 16 :element-type 'geometry_msgs-msg:Pose :initial-element (cl:make-instance 'geometry_msgs-msg:Pose))))
)

(cl:defclass BodyJoints (<BodyJoints>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <BodyJoints>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'BodyJoints)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kinect_v2-msg:<BodyJoints> is deprecated: use kinect_v2-msg:BodyJoints instead.")))

(cl:ensure-generic-function 'user_id-val :lambda-list '(m))
(cl:defmethod user_id-val ((m <BodyJoints>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kinect_v2-msg:user_id-val is deprecated.  Use kinect_v2-msg:user_id instead.")
  (user_id m))

(cl:ensure-generic-function 'tracked-val :lambda-list '(m))
(cl:defmethod tracked-val ((m <BodyJoints>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kinect_v2-msg:tracked-val is deprecated.  Use kinect_v2-msg:tracked instead.")
  (tracked m))

(cl:ensure-generic-function 'joints-val :lambda-list '(m))
(cl:defmethod joints-val ((m <BodyJoints>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kinect_v2-msg:joints-val is deprecated.  Use kinect_v2-msg:joints instead.")
  (joints m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <BodyJoints>) ostream)
  "Serializes a message object of type '<BodyJoints>"
  (cl:let* ((signed (cl:slot-value msg 'user_id)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'tracked))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'tracked))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'joints))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <BodyJoints>) istream)
  "Deserializes a message object of type '<BodyJoints>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'user_id) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'tracked) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'tracked) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  (cl:setf (cl:slot-value msg 'joints) (cl:make-array 16))
  (cl:let ((vals (cl:slot-value msg 'joints)))
    (cl:dotimes (i 16)
    (cl:setf (cl:aref vals i) (cl:make-instance 'geometry_msgs-msg:Pose))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<BodyJoints>)))
  "Returns string type for a message object of type '<BodyJoints>"
  "kinect_v2/BodyJoints")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'BodyJoints)))
  "Returns string type for a message object of type 'BodyJoints"
  "kinect_v2/BodyJoints")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<BodyJoints>)))
  "Returns md5sum for a message object of type '<BodyJoints>"
  "61535990ee807ee844649627b51297c2")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'BodyJoints)))
  "Returns md5sum for a message object of type 'BodyJoints"
  "61535990ee807ee844649627b51297c2")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<BodyJoints>)))
  "Returns full string definition for message of type '<BodyJoints>"
  (cl:format cl:nil "int32 user_id~%string tracked~%geometry_msgs/Pose[16] joints~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'BodyJoints)))
  "Returns full string definition for message of type 'BodyJoints"
  (cl:format cl:nil "int32 user_id~%string tracked~%geometry_msgs/Pose[16] joints~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <BodyJoints>))
  (cl:+ 0
     4
     4 (cl:length (cl:slot-value msg 'tracked))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'joints) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <BodyJoints>))
  "Converts a ROS message object to a list"
  (cl:list 'BodyJoints
    (cl:cons ':user_id (user_id msg))
    (cl:cons ':tracked (tracked msg))
    (cl:cons ':joints (joints msg))
))
