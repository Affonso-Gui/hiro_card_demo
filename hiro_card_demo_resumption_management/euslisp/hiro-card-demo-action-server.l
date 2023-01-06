(ros::roseus "action_server")
(load "hiro-card-demo-action-server-setup.l")


;; parameters
(defvar *coordinate-threshold* 50.0)
(defvar *coordinate-rot-threshold* (deg2rad 20.0))
(defvar *angle-vector-threshold* 1000.0)  ; don't consider angle-vector differences
(defvar *resume-timeout* 2.0)
(defvar *resume-motion-scale* 10.0)

(setq time-scale 1.2)
(setq *resume-motion-scale* (* *resume-motion-scale* time-scale))
(setq roseus_resume:*resume-time-scale* 10.0)
(setq roseus_resume:*resume-min-time* 1.0)
(setq roseus_resume:*intersection-resume-wait-time* 2.0)

(setq *epsilon-alist*
      (set-alist coordinates (list *coordinate-threshold* *coordinate-rot-threshold*)
                 *epsilon-alist*))


;; interrupt / resume callbacks
(defun preempt-motion (server goal)
  (set-red)
  (ros::ros-warn ";; Grasping Card: ~A" (send *ri* :get-robot-state :rarm-card))
  (ros::create-timer (/ roseus_resume:*intersection-resume-wait-time* 2.0)
    #'(lambda (event)
        (ros::ros-info ";; Reseting card grasping state")
        (send *ri* :reset-card-grasping-state))
    :groupname (*ri* . groupname) :oneshot t)
  (signals roseus_resume:on-interruption :status roseus_resume:*current-status*))

(defun resume-motion (server goal)
  (roseus_resume:reset-all-lstm-models)
  (set-blue))


;; define callbacks
(defun has-goal-request-cb (server request)
  *goal-request*)

(defun has-clear-request-cb (server request)
  ;; start impeadance to speed up later execution
  (send *ri* :start-impedance :rarm)
  *clear-request*)

(defun detect-execute-cb (server goal)
  (set-green)
  (detect-pose 3000 :wait t)
  t)

(defun pick-and-place-execute-cb (server goal)
  (set-green)
  (let ((cards (mapcar #'(lambda (c) (apply #'fix-card-pos c)) *goal-request*)))
    (unwind-protect
         (handler-case (prog1 (apply #'_pick&place cards)
                         (detect-pose 3000 :wait t))
           (assertion-error (e)
             (lisp::print-error-message e)
             nil))
      (setq *goal-request* nil))))

(defun retrieve-execute-cb (server goal)
  (set-green)
  (unwind-protect (retrieve-all-cards)
    (setq *clear-request* nil)))


;; create server instances
(instance roseus_bt:condition-node :init
          "/hiro_card_demo/wait_clear" hiro_card_demo_resumption_management::HasClearRequest
          :execute-cb 'has-clear-request-cb
          :groupname (*ri* . groupname))

(instance roseus_bt:condition-node :init
          "/hiro_card_demo/wait_goal" hiro_card_demo_resumption_management::HasGoalRequest
          :execute-cb 'has-goal-request-cb
          :groupname (*ri* . groupname))

(instance roseus_bt:automatic-resumable-action-node :init
          "/hiro_card_demo/detect" hiro_card_demo_resumption_management::DETECTAction
          :robot-interface *ri*
          :execute-cb 'detect-execute-cb
          :preempt-cb 'preempt-motion
          :resume-cb 'resume-motion
          :check-args (list :angle-vector-eps *angle-vector-threshold*
                            :robot-state-whitelist '(:rarm-card))
          :timeout *resume-timeout*
          :scale *resume-motion-scale*)

(instance roseus_bt:automatic-resumable-action-node :init
          "/hiro_card_demo/pick_and_place" hiro_card_demo_resumption_management::PICK_AND_PLACEAction
          :robot-interface *ri*
          :execute-cb 'pick-and-place-execute-cb
          :preempt-cb 'preempt-motion
          :resume-cb 'resume-motion
          :check-args (list :angle-vector-eps *angle-vector-threshold*
                            :robot-state-whitelist '(:rarm-card))
          :timeout *resume-timeout*
          :scale *resume-motion-scale*)

(instance roseus_bt:automatic-resumable-action-node :init
          "/hiro_card_demo/retrieve" hiro_card_demo_resumption_management::RETRIEVEAction
          :robot-interface *ri*
          :execute-cb 'retrieve-execute-cb
          :preempt-cb 'preempt-motion
          :resume-cb 'resume-motion
          :check-args (list :angle-vector-eps *angle-vector-threshold*
                            :robot-state-whitelist '(:rarm-card))
          :timeout *resume-timeout*
          :scale *resume-motion-scale*)


;; set rate
(ros::rate 50)

;; visualization
(make-irtviewer :draw-origin nil)
(set-green)

;; logging
;; (setq *cc-debug* t)
;; (ros::set-logger-level ros::*roserror*)
(log:set-logger-level :debug)

;; spin
;; (roseus_bt:spin)
