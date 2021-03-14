;; Auto-generated. Do not edit!


(when (boundp 'hrwros_msgs::Points_arrays)
  (if (not (find-package "HRWROS_MSGS"))
    (make-package "HRWROS_MSGS"))
  (shadow 'Points_arrays (find-package "HRWROS_MSGS")))
(unless (find-package "HRWROS_MSGS::POINTS_ARRAYS")
  (make-package "HRWROS_MSGS::POINTS_ARRAYS"))

(in-package "ROS")
;;//! \htmlinclude Points_arrays.msg.html


(defclass hrwros_msgs::Points_arrays
  :super ros::object
  :slots (_x_coordinates _y_coordinates ))

(defmethod hrwros_msgs::Points_arrays
  (:init
   (&key
    ((:x_coordinates __x_coordinates) (make-array 0 :initial-element 0 :element-type :char))
    ((:y_coordinates __y_coordinates) (make-array 0 :initial-element 0 :element-type :char))
    )
   (send-super :init)
   (setq _x_coordinates __x_coordinates)
   (setq _y_coordinates __y_coordinates)
   self)
  (:x_coordinates
   (&optional __x_coordinates)
   (if __x_coordinates (setq _x_coordinates __x_coordinates)) _x_coordinates)
  (:y_coordinates
   (&optional __y_coordinates)
   (if __y_coordinates (setq _y_coordinates __y_coordinates)) _y_coordinates)
  (:serialization-length
   ()
   (+
    ;; uint8[] _x_coordinates
    (* 1    (length _x_coordinates)) 4
    ;; uint8[] _y_coordinates
    (* 1    (length _y_coordinates)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint8[] _x_coordinates
     (write-long (length _x_coordinates) s)
     (princ _x_coordinates s)
     ;; uint8[] _y_coordinates
     (write-long (length _y_coordinates) s)
     (princ _y_coordinates s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint8[] _x_coordinates
   (let ((n (sys::peek buf ptr- :integer))) (incf ptr- 4)
     (setq _x_coordinates (make-array n :element-type :char))
     (replace _x_coordinates buf :start2 ptr-) (incf ptr- n))
   ;; uint8[] _y_coordinates
   (let ((n (sys::peek buf ptr- :integer))) (incf ptr- 4)
     (setq _y_coordinates (make-array n :element-type :char))
     (replace _y_coordinates buf :start2 ptr-) (incf ptr- n))
   ;;
   self)
  )

(setf (get hrwros_msgs::Points_arrays :md5sum-) "3b76707f47a52d893eafcf2404ef94bc")
(setf (get hrwros_msgs::Points_arrays :datatype-) "hrwros_msgs/Points_arrays")
(setf (get hrwros_msgs::Points_arrays :definition-)
      "uint8[] x_coordinates
uint8[] y_coordinates

")



(provide :hrwros_msgs/Points_arrays "3b76707f47a52d893eafcf2404ef94bc")


