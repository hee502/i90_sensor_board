; Auto-generated. Do not edit!


(cl:in-package i90_sensor_board-msg)


;//! \htmlinclude pos.msg.html

(cl:defclass <pos> (roslisp-msg-protocol:ros-message)
  ((fXPos
    :reader fXPos
    :initarg :fXPos
    :type cl:float
    :initform 0.0)
   (fYPos
    :reader fYPos
    :initarg :fYPos
    :type cl:float
    :initform 0.0)
   (fYawAngle
    :reader fYawAngle
    :initarg :fYawAngle
    :type cl:float
    :initform 0.0))
)

(cl:defclass pos (<pos>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <pos>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'pos)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name i90_sensor_board-msg:<pos> is deprecated: use i90_sensor_board-msg:pos instead.")))

(cl:ensure-generic-function 'fXPos-val :lambda-list '(m))
(cl:defmethod fXPos-val ((m <pos>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader i90_sensor_board-msg:fXPos-val is deprecated.  Use i90_sensor_board-msg:fXPos instead.")
  (fXPos m))

(cl:ensure-generic-function 'fYPos-val :lambda-list '(m))
(cl:defmethod fYPos-val ((m <pos>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader i90_sensor_board-msg:fYPos-val is deprecated.  Use i90_sensor_board-msg:fYPos instead.")
  (fYPos m))

(cl:ensure-generic-function 'fYawAngle-val :lambda-list '(m))
(cl:defmethod fYawAngle-val ((m <pos>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader i90_sensor_board-msg:fYawAngle-val is deprecated.  Use i90_sensor_board-msg:fYawAngle instead.")
  (fYawAngle m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <pos>) ostream)
  "Serializes a message object of type '<pos>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'fXPos))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'fYPos))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'fYawAngle))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <pos>) istream)
  "Deserializes a message object of type '<pos>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'fXPos) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'fYPos) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'fYawAngle) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<pos>)))
  "Returns string type for a message object of type '<pos>"
  "i90_sensor_board/pos")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'pos)))
  "Returns string type for a message object of type 'pos"
  "i90_sensor_board/pos")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<pos>)))
  "Returns md5sum for a message object of type '<pos>"
  "07182b1076047fa59cc732e44299f292")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'pos)))
  "Returns md5sum for a message object of type 'pos"
  "07182b1076047fa59cc732e44299f292")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<pos>)))
  "Returns full string definition for message of type '<pos>"
  (cl:format cl:nil "float32 fXPos~%float32 fYPos~%float32 fYawAngle~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'pos)))
  "Returns full string definition for message of type 'pos"
  (cl:format cl:nil "float32 fXPos~%float32 fYPos~%float32 fYawAngle~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <pos>))
  (cl:+ 0
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <pos>))
  "Converts a ROS message object to a list"
  (cl:list 'pos
    (cl:cons ':fXPos (fXPos msg))
    (cl:cons ':fYPos (fYPos msg))
    (cl:cons ':fYawAngle (fYawAngle msg))
))
