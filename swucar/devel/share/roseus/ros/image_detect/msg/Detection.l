;; Auto-generated. Do not edit!


(when (boundp 'image_detect::Detection)
  (if (not (find-package "IMAGE_DETECT"))
    (make-package "IMAGE_DETECT"))
  (shadow 'Detection (find-package "IMAGE_DETECT")))
(unless (find-package "IMAGE_DETECT::DETECTION")
  (make-package "IMAGE_DETECT::DETECTION"))

(in-package "ROS")
;;//! \htmlinclude Detection.msg.html


(defclass image_detect::Detection
  :super ros::object
  :slots (_pos _classes _scores ))

(defmethod image_detect::Detection
  (:init
   (&key
    ((:pos __pos) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:classes __classes) (make-array 0 :initial-element 0 :element-type :integer))
    ((:scores __scores) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _pos __pos)
   (setq _classes __classes)
   (setq _scores __scores)
   self)
  (:pos
   (&optional __pos)
   (if __pos (setq _pos __pos)) _pos)
  (:classes
   (&optional __classes)
   (if __classes (setq _classes __classes)) _classes)
  (:scores
   (&optional __scores)
   (if __scores (setq _scores __scores)) _scores)
  (:serialization-length
   ()
   (+
    ;; float32[] _pos
    (* 4    (length _pos)) 4
    ;; uint32[] _classes
    (* 4    (length _classes)) 4
    ;; float32[] _scores
    (* 4    (length _scores)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32[] _pos
     (write-long (length _pos) s)
     (dotimes (i (length _pos))
       (sys::poke (elt _pos i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; uint32[] _classes
     (write-long (length _classes) s)
     (dotimes (i (length _classes))
       (write-long (elt _classes i) s)
       )
     ;; float32[] _scores
     (write-long (length _scores) s)
     (dotimes (i (length _scores))
       (sys::poke (elt _scores i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32[] _pos
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _pos (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _pos i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; uint32[] _classes
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _classes (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _classes i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     ))
   ;; float32[] _scores
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _scores (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _scores i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;;
   self)
  )

(setf (get image_detect::Detection :md5sum-) "35145143adb0a6def78c300efff37377")
(setf (get image_detect::Detection :datatype-) "image_detect/Detection")
(setf (get image_detect::Detection :definition-)
      "float32[] pos       # 百分比位置 [x1, y1, x2, y2, _x1, _y1, _x2, _y2 ...]
uint32[] classes    # 类别
float32[] scores    # 分类自信值
")



(provide :image_detect/Detection "35145143adb0a6def78c300efff37377")


