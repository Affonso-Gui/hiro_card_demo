(ros::roseus "action_server")
(load "hiro-card-demo-action-server-setup.l")


;; define callbacks
(defun has-goal-request-cb (server request)
  *goal-request*)

(defun has-clear-request-cb (server request)
  ;; start impeadance to speed up later execution
  (send *ri* :start-impedance :rarm)
  *clear-request*)

(defun detect-execute-cb (server goal)
  (detect-pose 3000 :wait t)
  t)

(defun pick-and-place-execute-cb (server goal)
  (unwind-protect (apply #'pick&place *goal-request*)
    (setq *goal-request* nil)))

(defun retrieve-execute-cb (server goal)
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

(instance roseus_bt:action-node :init
          "/hiro_card_demo/detect" hiro_card_demo_resumption_management::DETECTAction
          :execute-cb 'detect-execute-cb
          :groupname (*ri* . groupname))

(instance roseus_bt:action-node :init
          "/hiro_card_demo/pick_and_place" hiro_card_demo_resumption_management::PICK_AND_PLACEAction
          :execute-cb 'pick-and-place-execute-cb
          :groupname (*ri* . groupname))

(instance roseus_bt:action-node :init
          "/hiro_card_demo/retrieve" hiro_card_demo_resumption_management::RETRIEVEAction
          :execute-cb 'retrieve-execute-cb
          :groupname (*ri* . groupname))


;; set rate
(ros::rate 50)

;; visualization
(make-irtviewer :draw-origin nil)
(set-green)

;; logging
;; (setq *cc-debug* t)
;; (ros::set-logger-level ros::*roserror*)
;; (log:set-logger-level :error)

;; spin
;; (roseus_bt:spin)
