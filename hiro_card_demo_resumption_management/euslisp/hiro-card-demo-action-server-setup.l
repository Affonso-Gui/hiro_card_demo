(ros::roseus "action_server")
(ros::load-ros-manifest "hiro_card_demo")
(ros::load-ros-package "hiro_card_demo_resumption_management")

(defvar *goal-request* nil)
(defvar *clear-request* nil)
(defparameter *enable-viewer* nil)

;; by using the same require key as in main.l we prevent from overwriting our changes
(require :hiro-interface "package://hiro_card_demo_resumption_management/euslisp/hironxjsk-card-interface.l")
(load "package://hiro_card_demo/src/main.l")
(load "package://roseus_resume/euslisp/lstm-intervention.l")
(load "package://roseus_resume/euslisp/hand-object-intervention.l")
(load "package://roseus_bt/euslisp/nodes.l")


(roseus_resume:install-lstm-model
 "/robot" :rarm "/docker/detic_segmentor/output/card/table/centroid" roseus_bt:cancel-action
 (*ri* . groupname))

(roseus_resume:install-object-intersection
 "/intersection/card" "/docker/detic_segmentor/output/card/table/centroid"
 (*ri* . groupname))

(roseus_resume::install-hand-intersection
"/intersection/right" :rarm (*ri* . groupname))
(roseus_resume::install-hand-intersection
 "/intersection/left" :larm (*ri* . groupname))

(send *ri* :start-card-monitoring
      :rarm "/docker/detic_segmentor/output/raw/centroid"
      :collision-model (make-cube 100.0 100.0 50.0))


(defun set-alist (k v alist &key (key) (test 'eq))
  (let ((cur-cons (assoc k alist :key key :test test)))
    (if cur-cons
        (progn (setf (cdr cur-cons) v) alist)
        (nconc alist (list (cons k v))))))

(defun reset-requests ()
  (setq *goal-request* nil)
  (setq *clear-request* nil))

(defun goal-cb (request)
  (ros::ros-info ";; Goal request received")
  (print-ros-msg (send request :cards))
  (flet ((get-card (card-msg)
           (list (send card-msg :x)
                 (send card-msg :y)
                 (plusp (send card-msg :flip)))))
    (let ((response (send request :response))
          (cards (mapcar #'get-card (send request :cards))))
      (setq *goal-request* cards)
      (send response :success t)
      response)))

(defun clear-cb (request)
  (ros::ros-info ";; Clear request received")
  (setq *clear-request* t)
  (send request :response))

;; scratch services
(ros::advertise-service "hiro_demo/goal" hiro_card_demo::Goal #'goal-cb
                        :groupname (*ri* . groupname))
(ros::advertise-service "hiro_demo/clear" std_srvs::Empty #'clear-cb
                        :groupname (*ri* . groupname))


;; dummy check functions
(defun always-true (&rest args) t)
(defun always-false (&rest args) nil)


;; visualization
(defun irt () (send *irtviewer* :draw-objects))

(defun set-red ()
  ;; interrupted
  (send *irtviewer* :change-background #f(1 0 0))
  (send *irtviewer* :redraw))

(defun set-green ()
  ;; normal execution
  (send *irtviewer* :change-background #f(0 1 0))
  (send *irtviewer* :redraw))

(defun set-blue ()
  ;; resumed
  (send *irtviewer* :change-background #f(0 0 1))
  (send *irtviewer* :redraw))


;; cleanup
(defun clean ()
  (setq *motion-checkpoint-list* nil)
  (reset-pose)
  (send *ri* :stop-grasp :larm)
  (dolist (ac roseus_bt::*action-list*)
    (send ac :set-val "GOAL" nil))
  (set-green))
