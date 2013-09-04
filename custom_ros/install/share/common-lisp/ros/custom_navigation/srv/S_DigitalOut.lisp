; Auto-generated. Do not edit!


(cl:in-package custom_navigation-srv)


;//! \htmlinclude S_DigitalOut-request.msg.html

(cl:defclass <S_DigitalOut-request> (roslisp-msg-protocol:ros-message)
  ((outputState
    :reader outputState
    :initarg :outputState
    :type (cl:vector cl:boolean)
   :initform (cl:make-array 4 :element-type 'cl:boolean :initial-element cl:nil)))
)

(cl:defclass S_DigitalOut-request (<S_DigitalOut-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <S_DigitalOut-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'S_DigitalOut-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name custom_navigation-srv:<S_DigitalOut-request> is deprecated: use custom_navigation-srv:S_DigitalOut-request instead.")))

(cl:ensure-generic-function 'outputState-val :lambda-list '(m))
(cl:defmethod outputState-val ((m <S_DigitalOut-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader custom_navigation-srv:outputState-val is deprecated.  Use custom_navigation-srv:outputState instead.")
  (outputState m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <S_DigitalOut-request>) ostream)
  "Serializes a message object of type '<S_DigitalOut-request>"
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if ele 1 0)) ostream))
   (cl:slot-value msg 'outputState))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <S_DigitalOut-request>) istream)
  "Deserializes a message object of type '<S_DigitalOut-request>"
  (cl:setf (cl:slot-value msg 'outputState) (cl:make-array 4))
  (cl:let ((vals (cl:slot-value msg 'outputState)))
    (cl:dotimes (i 4)
    (cl:setf (cl:aref vals i) (cl:not (cl:zerop (cl:read-byte istream))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<S_DigitalOut-request>)))
  "Returns string type for a service object of type '<S_DigitalOut-request>"
  "custom_navigation/S_DigitalOutRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'S_DigitalOut-request)))
  "Returns string type for a service object of type 'S_DigitalOut-request"
  "custom_navigation/S_DigitalOutRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<S_DigitalOut-request>)))
  "Returns md5sum for a message object of type '<S_DigitalOut-request>"
  "b64c19e3f60b552446035ef80f71381e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'S_DigitalOut-request)))
  "Returns md5sum for a message object of type 'S_DigitalOut-request"
  "b64c19e3f60b552446035ef80f71381e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<S_DigitalOut-request>)))
  "Returns full string definition for message of type '<S_DigitalOut-request>"
  (cl:format cl:nil "bool[4] outputState~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'S_DigitalOut-request)))
  "Returns full string definition for message of type 'S_DigitalOut-request"
  (cl:format cl:nil "bool[4] outputState~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <S_DigitalOut-request>))
  (cl:+ 0
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'outputState) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 1)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <S_DigitalOut-request>))
  "Converts a ROS message object to a list"
  (cl:list 'S_DigitalOut-request
    (cl:cons ':outputState (outputState msg))
))
;//! \htmlinclude S_DigitalOut-response.msg.html

(cl:defclass <S_DigitalOut-response> (roslisp-msg-protocol:ros-message)
  ((ret
    :reader ret
    :initarg :ret
    :type cl:fixnum
    :initform 0)
   (timestampSet
    :reader timestampSet
    :initarg :timestampSet
    :type cl:float
    :initform 0.0))
)

(cl:defclass S_DigitalOut-response (<S_DigitalOut-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <S_DigitalOut-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'S_DigitalOut-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name custom_navigation-srv:<S_DigitalOut-response> is deprecated: use custom_navigation-srv:S_DigitalOut-response instead.")))

(cl:ensure-generic-function 'ret-val :lambda-list '(m))
(cl:defmethod ret-val ((m <S_DigitalOut-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader custom_navigation-srv:ret-val is deprecated.  Use custom_navigation-srv:ret instead.")
  (ret m))

(cl:ensure-generic-function 'timestampSet-val :lambda-list '(m))
(cl:defmethod timestampSet-val ((m <S_DigitalOut-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader custom_navigation-srv:timestampSet-val is deprecated.  Use custom_navigation-srv:timestampSet instead.")
  (timestampSet m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <S_DigitalOut-response>) ostream)
  "Serializes a message object of type '<S_DigitalOut-response>"
  (cl:let* ((signed (cl:slot-value msg 'ret)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'timestampSet))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <S_DigitalOut-response>) istream)
  "Deserializes a message object of type '<S_DigitalOut-response>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'ret) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'timestampSet) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<S_DigitalOut-response>)))
  "Returns string type for a service object of type '<S_DigitalOut-response>"
  "custom_navigation/S_DigitalOutResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'S_DigitalOut-response)))
  "Returns string type for a service object of type 'S_DigitalOut-response"
  "custom_navigation/S_DigitalOutResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<S_DigitalOut-response>)))
  "Returns md5sum for a message object of type '<S_DigitalOut-response>"
  "b64c19e3f60b552446035ef80f71381e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'S_DigitalOut-response)))
  "Returns md5sum for a message object of type 'S_DigitalOut-response"
  "b64c19e3f60b552446035ef80f71381e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<S_DigitalOut-response>)))
  "Returns full string definition for message of type '<S_DigitalOut-response>"
  (cl:format cl:nil "int8 ret~%float64 timestampSet~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'S_DigitalOut-response)))
  "Returns full string definition for message of type 'S_DigitalOut-response"
  (cl:format cl:nil "int8 ret~%float64 timestampSet~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <S_DigitalOut-response>))
  (cl:+ 0
     1
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <S_DigitalOut-response>))
  "Converts a ROS message object to a list"
  (cl:list 'S_DigitalOut-response
    (cl:cons ':ret (ret msg))
    (cl:cons ':timestampSet (timestampSet msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'S_DigitalOut)))
  'S_DigitalOut-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'S_DigitalOut)))
  'S_DigitalOut-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'S_DigitalOut)))
  "Returns string type for a service object of type '<S_DigitalOut>"
  "custom_navigation/S_DigitalOut")