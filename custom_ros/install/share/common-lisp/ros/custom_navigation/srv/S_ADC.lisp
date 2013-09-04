; Auto-generated. Do not edit!


(cl:in-package custom_navigation-srv)


;//! \htmlinclude S_ADC-request.msg.html

(cl:defclass <S_ADC-request> (roslisp-msg-protocol:ros-message)
  ((subscribe
    :reader subscribe
    :initarg :subscribe
    :type cl:boolean
    :initform cl:nil)
   (adcChannels
    :reader adcChannels
    :initarg :adcChannels
    :type (cl:vector cl:boolean)
   :initform (cl:make-array 8 :element-type 'cl:boolean :initial-element cl:nil))
   (adcFreqAcq
    :reader adcFreqAcq
    :initarg :adcFreqAcq
    :type cl:fixnum
    :initform 0)
   (adcFreqPoll
    :reader adcFreqPoll
    :initarg :adcFreqPoll
    :type cl:fixnum
    :initform 0)
   (adcBufferSize
    :reader adcBufferSize
    :initarg :adcBufferSize
    :type cl:integer
    :initform 0))
)

(cl:defclass S_ADC-request (<S_ADC-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <S_ADC-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'S_ADC-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name custom_navigation-srv:<S_ADC-request> is deprecated: use custom_navigation-srv:S_ADC-request instead.")))

(cl:ensure-generic-function 'subscribe-val :lambda-list '(m))
(cl:defmethod subscribe-val ((m <S_ADC-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader custom_navigation-srv:subscribe-val is deprecated.  Use custom_navigation-srv:subscribe instead.")
  (subscribe m))

(cl:ensure-generic-function 'adcChannels-val :lambda-list '(m))
(cl:defmethod adcChannels-val ((m <S_ADC-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader custom_navigation-srv:adcChannels-val is deprecated.  Use custom_navigation-srv:adcChannels instead.")
  (adcChannels m))

(cl:ensure-generic-function 'adcFreqAcq-val :lambda-list '(m))
(cl:defmethod adcFreqAcq-val ((m <S_ADC-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader custom_navigation-srv:adcFreqAcq-val is deprecated.  Use custom_navigation-srv:adcFreqAcq instead.")
  (adcFreqAcq m))

(cl:ensure-generic-function 'adcFreqPoll-val :lambda-list '(m))
(cl:defmethod adcFreqPoll-val ((m <S_ADC-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader custom_navigation-srv:adcFreqPoll-val is deprecated.  Use custom_navigation-srv:adcFreqPoll instead.")
  (adcFreqPoll m))

(cl:ensure-generic-function 'adcBufferSize-val :lambda-list '(m))
(cl:defmethod adcBufferSize-val ((m <S_ADC-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader custom_navigation-srv:adcBufferSize-val is deprecated.  Use custom_navigation-srv:adcBufferSize instead.")
  (adcBufferSize m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <S_ADC-request>) ostream)
  "Serializes a message object of type '<S_ADC-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'subscribe) 1 0)) ostream)
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if ele 1 0)) ostream))
   (cl:slot-value msg 'adcChannels))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'adcFreqAcq)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'adcFreqAcq)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'adcFreqPoll)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'adcFreqPoll)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'adcBufferSize)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'adcBufferSize)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'adcBufferSize)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'adcBufferSize)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <S_ADC-request>) istream)
  "Deserializes a message object of type '<S_ADC-request>"
    (cl:setf (cl:slot-value msg 'subscribe) (cl:not (cl:zerop (cl:read-byte istream))))
  (cl:setf (cl:slot-value msg 'adcChannels) (cl:make-array 8))
  (cl:let ((vals (cl:slot-value msg 'adcChannels)))
    (cl:dotimes (i 8)
    (cl:setf (cl:aref vals i) (cl:not (cl:zerop (cl:read-byte istream))))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'adcFreqAcq)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'adcFreqAcq)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'adcFreqPoll)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'adcFreqPoll)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'adcBufferSize)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'adcBufferSize)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'adcBufferSize)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'adcBufferSize)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<S_ADC-request>)))
  "Returns string type for a service object of type '<S_ADC-request>"
  "custom_navigation/S_ADCRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'S_ADC-request)))
  "Returns string type for a service object of type 'S_ADC-request"
  "custom_navigation/S_ADCRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<S_ADC-request>)))
  "Returns md5sum for a message object of type '<S_ADC-request>"
  "6de67c1a66aa444be3767d62a6a225da")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'S_ADC-request)))
  "Returns md5sum for a message object of type 'S_ADC-request"
  "6de67c1a66aa444be3767d62a6a225da")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<S_ADC-request>)))
  "Returns full string definition for message of type '<S_ADC-request>"
  (cl:format cl:nil "bool subscribe~%bool[8] adcChannels~%uint16 adcFreqAcq~%uint16 adcFreqPoll~%uint32 adcBufferSize~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'S_ADC-request)))
  "Returns full string definition for message of type 'S_ADC-request"
  (cl:format cl:nil "bool subscribe~%bool[8] adcChannels~%uint16 adcFreqAcq~%uint16 adcFreqPoll~%uint32 adcBufferSize~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <S_ADC-request>))
  (cl:+ 0
     1
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'adcChannels) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 1)))
     2
     2
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <S_ADC-request>))
  "Converts a ROS message object to a list"
  (cl:list 'S_ADC-request
    (cl:cons ':subscribe (subscribe msg))
    (cl:cons ':adcChannels (adcChannels msg))
    (cl:cons ':adcFreqAcq (adcFreqAcq msg))
    (cl:cons ':adcFreqPoll (adcFreqPoll msg))
    (cl:cons ':adcBufferSize (adcBufferSize msg))
))
;//! \htmlinclude S_ADC-response.msg.html

(cl:defclass <S_ADC-response> (roslisp-msg-protocol:ros-message)
  ((ret
    :reader ret
    :initarg :ret
    :type cl:fixnum
    :initform 0))
)

(cl:defclass S_ADC-response (<S_ADC-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <S_ADC-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'S_ADC-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name custom_navigation-srv:<S_ADC-response> is deprecated: use custom_navigation-srv:S_ADC-response instead.")))

(cl:ensure-generic-function 'ret-val :lambda-list '(m))
(cl:defmethod ret-val ((m <S_ADC-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader custom_navigation-srv:ret-val is deprecated.  Use custom_navigation-srv:ret instead.")
  (ret m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <S_ADC-response>) ostream)
  "Serializes a message object of type '<S_ADC-response>"
  (cl:let* ((signed (cl:slot-value msg 'ret)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <S_ADC-response>) istream)
  "Deserializes a message object of type '<S_ADC-response>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'ret) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<S_ADC-response>)))
  "Returns string type for a service object of type '<S_ADC-response>"
  "custom_navigation/S_ADCResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'S_ADC-response)))
  "Returns string type for a service object of type 'S_ADC-response"
  "custom_navigation/S_ADCResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<S_ADC-response>)))
  "Returns md5sum for a message object of type '<S_ADC-response>"
  "6de67c1a66aa444be3767d62a6a225da")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'S_ADC-response)))
  "Returns md5sum for a message object of type 'S_ADC-response"
  "6de67c1a66aa444be3767d62a6a225da")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<S_ADC-response>)))
  "Returns full string definition for message of type '<S_ADC-response>"
  (cl:format cl:nil "int8 ret~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'S_ADC-response)))
  "Returns full string definition for message of type 'S_ADC-response"
  (cl:format cl:nil "int8 ret~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <S_ADC-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <S_ADC-response>))
  "Converts a ROS message object to a list"
  (cl:list 'S_ADC-response
    (cl:cons ':ret (ret msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'S_ADC)))
  'S_ADC-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'S_ADC)))
  'S_ADC-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'S_ADC)))
  "Returns string type for a service object of type '<S_ADC>"
  "custom_navigation/S_ADC")