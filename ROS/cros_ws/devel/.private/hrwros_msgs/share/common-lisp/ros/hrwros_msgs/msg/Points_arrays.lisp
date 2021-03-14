; Auto-generated. Do not edit!


(cl:in-package hrwros_msgs-msg)


;//! \htmlinclude Points_arrays.msg.html

(cl:defclass <Points_arrays> (roslisp-msg-protocol:ros-message)
  ((x_coordinates
    :reader x_coordinates
    :initarg :x_coordinates
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 0 :element-type 'cl:fixnum :initial-element 0))
   (y_coordinates
    :reader y_coordinates
    :initarg :y_coordinates
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 0 :element-type 'cl:fixnum :initial-element 0)))
)

(cl:defclass Points_arrays (<Points_arrays>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Points_arrays>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Points_arrays)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name hrwros_msgs-msg:<Points_arrays> is deprecated: use hrwros_msgs-msg:Points_arrays instead.")))

(cl:ensure-generic-function 'x_coordinates-val :lambda-list '(m))
(cl:defmethod x_coordinates-val ((m <Points_arrays>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hrwros_msgs-msg:x_coordinates-val is deprecated.  Use hrwros_msgs-msg:x_coordinates instead.")
  (x_coordinates m))

(cl:ensure-generic-function 'y_coordinates-val :lambda-list '(m))
(cl:defmethod y_coordinates-val ((m <Points_arrays>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hrwros_msgs-msg:y_coordinates-val is deprecated.  Use hrwros_msgs-msg:y_coordinates instead.")
  (y_coordinates m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Points_arrays>) ostream)
  "Serializes a message object of type '<Points_arrays>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'x_coordinates))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream))
   (cl:slot-value msg 'x_coordinates))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'y_coordinates))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream))
   (cl:slot-value msg 'y_coordinates))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Points_arrays>) istream)
  "Deserializes a message object of type '<Points_arrays>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'x_coordinates) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'x_coordinates)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream)))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'y_coordinates) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'y_coordinates)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Points_arrays>)))
  "Returns string type for a message object of type '<Points_arrays>"
  "hrwros_msgs/Points_arrays")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Points_arrays)))
  "Returns string type for a message object of type 'Points_arrays"
  "hrwros_msgs/Points_arrays")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Points_arrays>)))
  "Returns md5sum for a message object of type '<Points_arrays>"
  "3b76707f47a52d893eafcf2404ef94bc")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Points_arrays)))
  "Returns md5sum for a message object of type 'Points_arrays"
  "3b76707f47a52d893eafcf2404ef94bc")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Points_arrays>)))
  "Returns full string definition for message of type '<Points_arrays>"
  (cl:format cl:nil "uint8[] x_coordinates~%uint8[] y_coordinates~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Points_arrays)))
  "Returns full string definition for message of type 'Points_arrays"
  (cl:format cl:nil "uint8[] x_coordinates~%uint8[] y_coordinates~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Points_arrays>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'x_coordinates) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 1)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'y_coordinates) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 1)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Points_arrays>))
  "Converts a ROS message object to a list"
  (cl:list 'Points_arrays
    (cl:cons ':x_coordinates (x_coordinates msg))
    (cl:cons ':y_coordinates (y_coordinates msg))
))
