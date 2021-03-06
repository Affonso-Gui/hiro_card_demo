(require :hiro-interface "package://hrpsys_ros_bridge_tutorials/euslisp/hironxjsk-interface.l")
(load "models/room73b2-foldable-desk-object.l")
(load "models/arrow-object.l")

(defmethod HIRONXJSK-robot
  (:reset-pose ()
    (send self :angle-vector #f(-7.37441 -4.85583 -103.028 -77.7359 11.3909 -15.6836 13.4646 1.82886 -88.6255 119.881 13.3115 3.92436 0.0 2.135382e-18 29.9943)))
  (:standby (x y &optional flip)
    (send self :reset-pose)
    (if flip (send self :rarm-wrist-r :joint-angle 163))
    (let* ((mid (midpoint 0.5
                          (send (send *hironxjsk* :rarm-end-coords) :worldpos)
                          (send (send *hironxjsk* :larm-end-coords) :worldpos)))
           (end (v+ (float-vector x y) #f(-70 70)))
           (diff (concatenate float-vector (v- end (subseq mid 0 2)) #f(0))))
      (assert-reset (send *hironxjsk* :rarm :move-end-pos diff :world))
      (send *hironxjsk* :larm :move-end-pos diff :world))))

(setq table (room73b2-foldable-desk))
(send table :locate #f(400 0.0 -800.0) :world)
(setq deck (make-cube 100 60 26))
(send deck :set-color :red)
(hironxjsk-init :objects (list table))
(objects (list *hironxjsk* table deck))

;; RHAND-END-COORDS
(setq rhand-end-coords (send (arrow) :newcoords (send (send *hironxjsk* :rarm-end-coords) :copy-worldcoords)))
(send rhand-end-coords :translate #f(0 0 -70))
(send (send *hironxjsk* :rarm-end-coords) :assoc rhand-end-coords)

;; Time related
(defvar time-scale 1.0)

;; Robot motion
(defun send-robot (time &key wait)
  (prog1
      (send *ri* :angle-vector (send *hironxjsk* :angle-vector)
            (* time time-scale)
            nil 0 :scale time-scale :min-time time-scale)
    (when wait (send *ri* :wait-interpolation))
    (when (send *ri* :simulation-modep) (irt))))

(defun send-robot-sequence (avs times &key wait)
  (prog1
      (send *ri* :angle-vector-sequence avs
            (mapcar #'(lambda (val) (* val time-scale)) times)
            nil 0 :scale time-scale :min-time time-scale)
    (when wait (send *ri* :wait-interpolation))
    (when (send *ri* :simulation-modep) (irt))))

(defun send-hand-viewer (hand)
  (flet ((get-joint-list (hand)
           (let (acc)
             (dotimes (i 4) (push (read-from-string (format nil "~a_joint~a" hand i)) acc))
             (nreverse acc))))
    (let ((joint-list (case hand
                        (:hands (append (get-joint-list :rhand) (get-joint-list :lhand)))
                        ((:rhand :lhand) (get-joint-list hand))
                        (t (error ";; No such hand: ~A~%." hand))))
          (av (send *ri* :hand-angle-vector hand)))
      (map float-vector #'(lambda (joint angle) (send *hironxjsk* joint :joint-angle angle))
           joint-list av))))

(defun send-hand (hand av tm)
  (prog1
      (send *ri* :hand-angle-vector hand av (* tm time-scale))
    (send-hand-viewer hand)))

(defun send-hand-width (hand width &key time)
  (prog1
      (send *ri* :hand-width hand width :time (* time time-scale))
    (send-hand-viewer hand)))

(defun sleep-time-scale (tm)
  (unix:usleep (truncate (* 1000 time-scale tm))))

;; Utility function
(defun irt () (send *irtviewer* :draw-objects))

(defun waist-transform (coords)
  (send (send (send *hironxjsk* :torso-end-coords) :copy-worldcoords)
        :transform coords))

(defun send-viewer ()
  (send *hironxjsk* :angle-vector (send *ri* :state :angle-vector))
  (send *irtviewer* :draw-objects))

(defun send-reference ()
  (send *hironxjsk* :angle-vector (send *ri* :state :reference-vector))
  (send *irtviewer* :draw-objects))

(defun send-rarm (vec)
  (send *hironxjsk* :rarm :angle-vector vec)
  (send *hironxjsk* :angle-vector))

(defun reset-pose (&optional (time 5000) &key wait)
  (prog1
    (send *hironxjsk* :reset-pose)
    (send-hand-width :rhand 20 :time time)
    (send-robot time :wait wait)
    (send *irtviewer* :draw-objects)))

(defun assert-reset (form &rest msg)
  (unless form
    (warning-message 1 "Assertation failed!~%")
    (reset)))

;; Pick & Place
(defun grasp-deck (&key wait)
  (send *ri* :hand-effort :lhand #f(3 3 50 3))
  (send-hand :lhand #f(-38.9986 -90.4 -45 92.2) 750) ;; grasp deck
  (if wait (sleep-time-scale 1000)))

(defun pick-card ()
  ;; push card
  (send-hand :lhand
        (v+ #f(-10 -10 0 0) (send *ri* :hand-angle-vector :lhand)) 750)
  (send *ri* :hand-effort :lhand '(100 100 nil nil))
  (sleep-time-scale 1000)

  (send *ri* :hand-effort :lhand '(nil nil nil 1))
  (send-hand :lhand #f(-55 -11.8 -25 30) 800) ;; push card
  (sleep-time-scale 750)

  (send *ri* :hand-effort :lhand 3)

  ;; grasp card
  (send-hand-width :rhand 20 :time 1000)
  (assert-reset (send *hironxjsk* :rarm :move-end-pos #f(0 0 -60)))
  (send-robot 1000 :wait t)

  (send *ri* :hand-effort :rhand 100)
  (send-hand-width :rhand -2 :time 500)
  (sleep-time-scale 1000)

  (send *ri* :stop-grasp :larm))

(defun place-card (x y &optional flip)
  (when flip
    (send *hironxjsk* :rarm-wrist-r :joint-angle -15.6836)
    (setq x (- x 63)))

  (let ((v1 (progn
              (assert-reset (send *hironxjsk* :rarm :move-end-pos #f(0 -25 -30) :world))
              (send *hironxjsk* :larm :move-end-pos #f(-20 25 -20) :world)))

        (v2 (send *hironxjsk* :rarm
                  :inverse-kinematics (waist-transform (make-coords :pos (float-vector x y 6.151) :rpy #f(0.115 0.009 1.178)))
                  :move-target rhand-end-coords))
        (v3 (send *hironxjsk* :rarm
                  :inverse-kinematics (waist-transform (make-coords :pos (float-vector x y -93.85) :rpy #f(0.189 -0.011 0.482)))
                  :move-target rhand-end-coords))
        (v4 (send *hironxjsk* :rarm
                  :inverse-kinematics (waist-transform (make-coords :pos (float-vector
                                                                          (if flip
                                                                            (- x 7.742)
                                                                            (+ x 7.742))
                                                                          (- y 14.636) -120.777)
                                                                    :rpy
                                                                    (if flip
                                                                        #f(-0.169 -0.023 0.261)
                                                                        #f(0.169 -0.067 0.253))))
                  :move-target rhand-end-coords)))

    (assert-reset (every #'vectorp (list v1 v2 v3 v4)))
    (when flip (setf (elt v1 5) 120)) ;; rarm-wrist-r

    ;; place on table
    (send-robot-sequence (list v1 v2 v3 v4) (list 500 750 1000 750))
    (sleep-time-scale 1000)
    (grasp-deck)
    (sleep-time-scale 1750)
    (send-hand :rhand #f(25 -90.5 20.5986 -26.4) 750)
    (send *ri* :wait-interpolation)
    (let ((card-obj (make-cube 100 60 1)))
      (send card-obj :set-color (if flip :white :red))
      (send card-obj :locate (float-vector (elt (send rhand-end-coords :worldpos) 0)
                                           (elt (send rhand-end-coords :worldpos) 1)
                                           -100))
      (objects card-obj))
    (sleep-time-scale 250)))

(defun clear-table ()
  (objects (list *hironxjsk* table deck)))

(defun go-standby (&optional x y flip)
  (let ((v1 (send *hironxjsk* :rarm :move-end-pos #f(0 0 30)))
        (v2 (if (and x y)
              (send *hironxjsk* :standby x y flip)
              (send *hironxjsk* :reset-pose))))
  (send-hand-width :rhand 30 :time 1500)
  (send-robot-sequence (list v1 v2) (list 1000 1500))))

(defun _pick&place (&rest cards)
  (flet ((fix-card-pos (x y &optional flip)
           ;; switch camera coords to robot coords
           ;; scratch window: -210~210 x -70~140
           (list (max 200 (+ y 340)) x flip)))
    (when cards
      (setq cards (mapcar #'(lambda (c) (apply #'fix-card-pos c)) cards))
      (send* *hironxjsk* :standby (car cards))
      (send-robot 1500)
      (grasp-deck :wait t))

    (dotimes (i (length cards))
      (pick-card)
      (apply #'place-card (nth i cards))
      (apply #'go-standby (nth (1+ i) cards)))))


;; Demo Function
(defun pick&place (&rest cards)
  (let ((res (catch 0 (apply #'_pick&place cards) t)))
    (unless res (reset-pose 5000 :wait t))
    res))

;; Main Loop
(format t "Reseting Pose ...~%")
(reset-pose 5000 :wait t)
(send *ri* :stop-grasp :larm)
(send deck :locate (send (send *hironxjsk* :larm-end-coords) :worldpos))
(send (send *hironxjsk* :larm-end-coords) :assoc deck)
(irt)

(format t ";; (pick&place '(40 40) '(-30 40) '(-100 40 t))~%")
(format t ";; (clear-table)~%")
