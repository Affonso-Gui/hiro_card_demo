(load "package://hrpsys_ros_bridge_tutorials/euslisp/hironxjsk-interface.l")

(defmethod hironxjsk-robot
  (:rhand ()
    (list rhand_joint0_link_lk rhand_joint1_link_lk rhand_joint2_link_lk rhand_joint3_link_lk))
  (:lhand ()
    (list lhand_joint0_link_lk lhand_joint1_link_lk lhand_joint2_link_lk lhand_joint3_link_lk)))

;; Register the card grasping condition in the robot-state
(defclass hironxjsk-card-interface
    :super hironxjsk-interface)
(defmethod hironxjsk-card-interface
  (:init (&rest args)
    (send-super* :init args)
    (send self :reset-card-grasping-state)
    self)
  (:hironxjsk-card-grasping-state-callback (arm collision-model msg)
    (let* ((av-save (send robot :angle-vector))
           (endc (progn (send robot :angle-vector (send self :state :potentio-vector))
                        (send robot arm :end-coords)))
           (key (intern (format nil "~A-CARD" (string arm)) *keyword-package*))
           (hand (case arm
                   (:rarm :rhand)
                   (:larm :lhand)
                   (t (error value-error "unknown arm type: ~A" arm)))))
      (dolist (pose (send msg :poses))
        (send collision-model :newcoords (ros::tf-pose->coords pose))
        ;; eus model origin is in the WAIST frame
        ;; $ rosrun tf tf_echo odom WAIST
        ;; At time 1672737042.787
        ;; - Translation: [0.000, 0.000, 0.800]
        ;; - Rotation: [0.000, 0.000, 0.000]
        (send collision-model :translate #f(0 0 -800) :world)
        (dolist (lk (send robot hand))
          (when (= (user::collision-check lk collision-model) 1)
            (send self :set-robot-state1 key t)
            (when (boundp '*hironxjsk*)
              (let ((tint (get-tint-mark (get-tint (send *hironxjsk* arm :angle-vector)))))
                ;; check in *angle-vector-tint*
                (when (and (not tint) (assoc arm *angle-vector-tint*))
                  (setq tint (get-tint-mark *angle-vector-tint*)))
                (when tint
                  (if (assoc arm *gripper-tint*)
                      ;; don't use setf to avoid having to clone in motion-checkpoint
                      (setq *gripper-tint* (acons arm tint (remove arm *gripper-tint* :key #'car)))
                      ;; add to list
                      (setq *gripper-tint* (acons arm tint *gripper-tint*)))))
            (return-from :hironxjsk-card-grasping-state-callback t)))))))
  (:start-card-monitoring (arm topic &key
      (collision-range 100.0)
      (collision-model (make-cube collision-range collision-range collision-range)))
    (let ((ros::*track-tint* nil))
      (ros::subscribe topic geometry_msgs::PoseArray
                      #'send self :hironxjsk-card-grasping-state-callback arm collision-model
                      :groupname groupname)))
  (:stop-card-monitoring (topic)
    (ros::unsubscribe topic))
  (:reset-card-grasping-state (&optional (arm :arms))
    (dolist (a (if (eq arm :arms) '(:larm :rarm) (list arm)))
      (setq *gripper-tint* (remove a *gripper-tint* :key #'car))
      (if (eql a :rarm) (send self :set-robot-state1 :rarm-card nil))
      (if (eql a :larm) (send self :set-robot-state1 :larm-card nil)))
    t)
  (:hand-angle-vector (hand &optional av (tm 1000))
    (let ((arm (case hand
                 (:rhand :rarm)
                 (:lhand :larm)
                 (:hands :arms))))
      (send self :reset-card-grasping-state arm)
      (send-super :hand-angle-vector hand av tm))))


;; use card-interface and don't tract tint markings
(defun hironxjsk-init (&rest args)
  (let ((ros::*track-tint* nil))
    (if (not (boundp '*ri*))
        (setq *ri* (instance* hironxjsk-card-interface :init args)))
    (if (not (boundp '*hironxjsk*))
        (setq *hironxjsk* (instance hironxjsk-sensor-robot :init)))
    (send *ri* :spin-once)
    (send *ri* :draw-objects)
    ;; read initial robot state
    (send *hironxjsk* :angle-vector (send *ri* :state :potentio-vector))
    ;; return robot instance
    *hironxjsk*))
