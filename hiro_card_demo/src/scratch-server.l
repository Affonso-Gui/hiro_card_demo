#! /usr/bin/env roseus

(require :hiro-card-demo "main.l")
(ros::load-ros-manifest "hiro_card_demo")

;; Service Callbacks
(defun goal-cb (request)
  ;; (print-ros-msg request)
  (print-ros-msg (send request :cards))
  (flet ((get-card (card-msg)
           (list (send card-msg :x)
                 (send card-msg :y)
                 (plusp (send card-msg :flip)))))
    (let* ((response (send request :response))
           (cards (mapcar #'get-card (send request :cards)))
           (res (apply #'pick&place cards)))
      (send response :success res)
      response)))

(defun clear-cb (request)
  (clear-table)
  (send request :response))

(ros::advertise-service "hiro_demo/goal" hiro_card_demo::Goal #'goal-cb)
(ros::advertise-service "hiro_demo/clear" std_srvs::Empty #'clear-cb)

(format t "Starting service at /hiro_demo/goal ...~%")

(ros::spin)
