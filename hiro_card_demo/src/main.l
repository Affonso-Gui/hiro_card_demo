(require :hiro-interface "package://hrpsys_ros_bridge_tutorials/euslisp/hironxjsk-interface.l")
(load "models/room73b2-foldable-desk-object.l")
(load "models/arrow-object.l")

(defvar *enable-viewer* t)

(defmethod HIRONXJSK-robot
  (:reset-pose ()
    (send self :angle-vector #f(-7.37441 -4.85583 -103.028 -77.7359 11.3909 -15.6836 13.4646 1.82886 -88.6255 119.881 13.3115 3.92436 0.0 5.0 60.0)))
  (:detect-pose ()
    (send self :angle-vector #f(-26.1343 -19.5059 -91.3939 -84.0743 29.3296 -15.8469 31.3788 -13.3809 -74.5263 122.956 28.0565 14.8722 0.0 5.0 60.0)))
  (:standby (x y &optional flip)
    (send self :reset-pose)
    (if flip (send self :rarm-wrist-r :joint-angle 163))
    (let* ((mid (midpoint 0.5
                          (send (send *hironxjsk* :rarm-end-coords) :worldpos)
                          (send (send *hironxjsk* :larm-end-coords) :worldpos)))
           (end (v+ (float-vector x y) #f(-70 70)))
           (diff (concatenate float-vector (v- end (subseq mid 0 2)) #f(0))))
      (assert (send *hironxjsk* :rarm :move-end-pos diff :world))
      (send *hironxjsk* :larm :move-end-pos diff :world)))
  (:maybe-standby (&optional x y flip)
    (if (and x y)
        (send self :standby x y flip)
        (send self :reset-pose))))

(setq table (room73b2-foldable-desk))
(send table :locate #f(400 0.0 -800.0) :world)
(setq deck (make-cube 100 60 26))
(send deck :set-color :red)
(hironxjsk-init :objects (list table))
(when *enable-viewer* (objects (list *hironxjsk* table deck)))

;; RHAND-END-COORDS
(setq rhand-end-coords (send (arrow) :newcoords (send (send *hironxjsk* :rarm-end-coords) :copy-worldcoords)))
(send rhand-end-coords :translate #f(0 0 -70))
(send (send *hironxjsk* :rarm-end-coords) :assoc rhand-end-coords)

;; Time related
(defvar time-scale 1.0)

;; Robot motion
(defun send-robot (time &key wait)
  (ros::ros-warn "MOTION: rarm-card ~A" (send *ri* :get-robot-state :rarm-card))
  (prog1
      (send *ri* :angle-vector (send *hironxjsk* :angle-vector)
            (* time time-scale)
            nil 0 :scale time-scale :min-time time-scale)
    (when wait (send *ri* :wait-interpolation))
    (when (and *enable-viewer* (send *ri* :simulation-modep))
      (irt))))

(defun send-robot-sequence (avs times &key wait)
  (prog1
      (send *ri* :angle-vector-sequence avs
            (mapcar #'(lambda (val) (* val time-scale)) times)
            nil 0 :scale time-scale :min-time time-scale)
    (when wait (send *ri* :wait-interpolation))
    (when (and *enable-viewer* (send *ri* :simulation-modep))
      (irt))))

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
  (let ((tm (* tm time-scale 1e-3))
        now)
    (while (> tm 0)
      (lisp::atomic
          (progn
            (setq now (ros::time-now))
            (send *ri* :spin-once)
            (ros::sleep)
            (decf tm (send (ros::time- (ros::time-now) now) :to-sec)))
        (interruption ())))))

;; Utility function
(defun irt () (send *irtviewer* :draw-objects))

(defun waist-transform (coords)
  (send (send (send *hironxjsk* :torso-end-coords) :copy-worldcoords)
        :transform coords))

(defun send-viewer ()
  (send *hironxjsk* :angle-vector (send *ri* :state :angle-vector))
  (when *enable-viewer* (send *irtviewer* :draw-objects)))

(defun send-reference ()
  (send *hironxjsk* :angle-vector (send *ri* :state :reference-vector))
  (when *enable-viewer* (send *irtviewer* :draw-objects)))

(defun send-rarm (vec)
  (send *hironxjsk* :rarm :angle-vector vec)
  (send *hironxjsk* :angle-vector))

(defun reset-pose (&optional (time 5000) &key wait)
  (prog1
    (send *hironxjsk* :reset-pose)
    (send-hand-width :rhand 20 :time time)
    (send-robot time :wait wait)
    (when *enable-viewer* (send *irtviewer* :draw-objects))))

(defun detect-pose (&optional (time 5000) &key wait)
  (prog1
      (send *hironxjsk* :detect-pose)
    (send-hand-width :rhand 20 :time time)
    (send-robot time :wait wait)
    (when *enable-viewer* (send *irtviewer* :draw-objects))))

;; Pick & Place
(defun grasp-deck (&key wait)
  (send *ri* :hand-effort :lhand #f(3 3 50 10))
  (send-hand :lhand #f(-38.9986 -90.4 -45 92.2) 750) ;; grasp deck
  (if wait (sleep-time-scale 1000)))

(defun pick-card ()
  (log:log-info "PICK-CARD")
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
  (assert (send *hironxjsk* :rarm :move-end-pos #f(0 0 -60)))
  (send-robot 1000 :wait t)

  (send *ri* :hand-effort :rhand 100)
  (send-hand-width :rhand -2 :time 500)
  (sleep-time-scale 1000)

  (send *ri* :stop-grasp :larm))

(defun draw-card ()
  (grasp-deck :wait t)
  (pick-card)
  (send *hironxjsk* :rarm :move-end-pos #f(0 -25 0) :world)
  (send-robot 1000 :wait t)
  (send *hironxjsk* :reset-pose)
  (send-robot 1000)
  (send-hand-width :rhand 20 :time 1000))

(defun place-card (x y &optional flip)
  (log:log-info "PLACE-CARD")
  (when flip
    (send *hironxjsk* :rarm-wrist-r :joint-angle -15.6836)
    (setq x (- x 63)))
  (let ((v1 (progn
              (assert (send *hironxjsk* :rarm :move-end-pos #f(0 -25 -30) :world))
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
                                                                          (- y 14.636) -116.777)
                                                                    :rpy
                                                                    (if flip
                                                                        #f(-0.169 -0.023 0.261)
                                                                        #f(0.169 -0.067 0.253))))
                  :move-target rhand-end-coords)))

    (assert (every #'vectorp (list v1 v2 v3 v4)))
    (when flip (setf (elt v1 5) 120)) ;; rarm-wrist-r

    ;; place on table
    (send-robot-sequence (list v1 v2 v3 v4) (list 500 750 1000 750))
    (sleep-time-scale 1000)
    (grasp-deck)
    (sleep-time-scale 1750)
    (send-hand :rhand #f(25 -90.5 20.5986 -26.4) 750)
    (send *ri* :wait-interpolation)
    (when *enable-viewer*
      (let ((card-obj (make-cube 100 60 1)))
        (send card-obj :set-color (if flip :white :red))
        (send card-obj :locate (float-vector (elt (send rhand-end-coords :worldpos) 0)
                                             (elt (send rhand-end-coords :worldpos) 1)
                                             -100))
        (objects card-obj)))
    (sleep-time-scale 250)))

(defun retrieve-card (x y &optional flip)
  (log:log-info "RETRIEVE CARD")
  (setq flip nil)  ;; only kept as an argument for consistency

  ;; this function needs impedance control
  ;; it is the responsability of the caller to stop it after execution
  (send *ri* :start-impedance :rarm)

  ;; move to card
  (let* ((v1 (send *hironxjsk* :standby x y))
         (v2 (send *hironxjsk* :rarm
                   :inverse-kinematics (waist-transform (make-coords :pos (float-vector
                                                                           (- x 10.0)
                                                                           (- y 2.0) -80.777)
                                                                     :rpy #f(0.169 -0.067 0.253)))
                   :move-target rhand-end-coords))
         (v3 (send *hironxjsk* :rarm :move-end-pos #f(0 0 -40) :world)))

    (assert (every #'vectorp (list v1 v2 v3)))
    (send-hand-width :rhand 0 :time 750)
    (send-robot-sequence (list v1 v2) (list 1500 1500) :wait t)

    (send-hand :rhand #f(25 -90.5 20.5986 -26.4) 750)
    (send *hironxjsk* :angle-vector v3)
    (send-robot 1500 :wait t))

  ;; drag to border
  (log:log-info "DRAG")
  (send *hironxjsk* :angle-vector #f(-14.0598 -16.1336 -83.7391 0.662583 12.4256 38.7536 36.3125 -15.3647 -68.4375 122.106 34.4121 13.9225 -25.0 5.0 60.0))
  (send-robot 3000 :wait t)

  ;; grasp card
  (send-hand :rhand #f(35.2366 0.0 20.3986 -26.5) 1000) ;; second to zero

  (send *ri* :hand-effort :rhand 3)
  (send *hironxjsk* :angle-vector #f(-24.3646 -38.6116 -51.6646 -20.0808 24.6508 43.8082 36.3125 -15.3647 -68.4375 122.106 34.4121 13.9225 -25.0 5.0 60.0))
  (send-robot 2000 :wait t)

  (send *ri* :start-grasp :rhand :effort 5)
  (sleep-time-scale 1000)
  (send *hironxjsk* :angle-vector #f(-26.2288 -47.4995 -38.4989 -18.6733 21.2299 47.2583 36.3125 -15.3647 -68.4375 122.106 34.4121 13.9225 -25.0 5.0 60.0))
  (send-robot 1000 :wait t)

  (send *ri* :start-grasp :rhand :effort 100)
  (send-hand-width :rhand -2 :time 500)

  ;; lift & reset-pose
  (log:log-info "LIFT")
  (let ((v1 #f(-26.1234 -44.8901 -49.6257 -21.6042 47.9646 43.0482 36.3125 -15.3647 -68.4375 122.106 34.4121 13.9225 -25.0 5.0 60.0))
        (v2 #f(-6.25666 -5.70116 -106.475 -78.2377 11.2569 -20.1883 13.4646 1.82886 -88.6255 119.881 13.3115 3.92436 0.0 5.0 60.0)))
    (send-robot-sequence (list v1 v2) (list 1000 2000))
    (sleep-time-scale 1000)
    (send *ri* :stop-grasp :larm)
    (send *ri* :wait-interpolation))

  ;; place on deck
  (unwind-protect
       (progn
         (send *ri* :stop-collision-detection)
         (send *hironxjsk* :angle-vector #f(19.2219 11.1853 -116.308 -69.9534 -13.8515 -20.1379 13.4646 1.82886 -88.6255 119.881 13.3115 3.92436 0.0 5.0 60.0))
         (send-robot 2000 :wait t)
         (send *hironxjsk* :angle-vector #f(23.0086 5.51433 -123.968 -40.8259 5.39099 -35.6239 13.4646 1.82886 -88.6255 119.881 13.3115 3.92436 0.0 5.0 60.0))
         (send-robot 2000 :wait t))
    (send *ri* :start-collision-detection))

  (send *ri* :hand-effort :lhand #f(3 3 50 10))
  (send-hand :lhand #f(-38.9986 -90.4 -45 92.2) 750)
  (sleep-time-scale 1000)
  (send-hand-width :rhand 2 :time 500)
  (send *ri* :hand-effort :rhand 3)

  ;; move away
  (send *hironxjsk* :angle-vector #f(2.89197 -8.10794 -129.146 -54.3726 29.6771 -38.7213 13.4646 1.82886 -88.6255 119.881 13.3115 3.92436 0.0 5.0 60.0))
  (send-robot 1500)
  (grasp-deck)
  (send *ri* :hand-effort :rhand 100)
  (reset-pose 2000 :wait t))

(defun clear-table ()
  (when *enable-viewer*
    (objects (list *hironxjsk* table deck))))

(defun go-standby (&optional x y flip &key wait)
  (let ((v1 (send *hironxjsk* :rarm :move-end-pos #f(0 0 30)))
        (v2 (send *hironxjsk* :maybe-standby x y flip)))
  (send-hand-width :rhand 30 :time 1500)
  (send-robot-sequence (list v1 v2) (list 1000 1500) :wait wait)))

(defun fix-card-pos (x y &optional flip)
  ;; switch camera coords to robot coords
  ;; scratch window: -210~210 x -70~140
  (list (max 200 (+ y 340)) x flip))

(defun fix-msg-pos (pose)
  ;; geometry_msgs/Pose to '(x y)
  (list (* (send (send pose :position) :x) 1000)
        (* (send (send pose :position) :y) 1000)
        nil))

(defun _pick&place (&rest cards)
  (if (not cards) (return-from _pick&place nil))
  (send* *hironxjsk* :standby (car cards))
  (send-robot 1500)
  (grasp-deck :wait t)

  (dotimes (i (length cards))
    (pick-card)
    (apply #'place-card (nth i cards))
    (apply #'go-standby (nth (1+ i) cards)))
  t)

(defun _retrieve (&rest cards)
  (if (not cards) (return-from _retrieve nil))
  (unwind-protect
       (progn
         (send *ri* :start-impedance :rarm)
         (dotimes (i (length cards))
           (apply #'retrieve-card (nth i cards)))
         (reset-pose 3000 :wait t)
         t)
    (send *ri* :stop-impedance-no-wait :rarm)))


;; Demo Function
(defmacro with-assertion-catch (form)
  (let ((result (gensym)))
    `(handler-case ,form
       (assertion-error (e)
         (lisp::print-error-message e)
         (ros::ros-warn "Aborting task and returning to home position...")
         (reset-pose 5000 :wait t)
         nil))))

(defun pick&place (&rest cards)
  (setq cards (mapcar #'(lambda (c) (apply #'fix-card-pos c)) cards))
  (with-assertion-catch
      (prog1 (apply #'_pick&place cards)
        (reset-pose 3000 :wait t))))

(defun retrieve (&rest cards)
  (setq cards (mapcar #'(lambda (c) (apply #'fix-card-pos c)) cards))
  (with-assertion-catch (apply #'_retrieve cards)))

(defun retrieve-all-cards ()
  (let* ((msg (one-shot-subscribe "/docker/detic_segmentor/output/card/table/centroid" geometry_msgs::PoseArray))
         (cards (mapcar #'fix-msg-pos (send msg :poses))))
    (with-assertion-catch (apply #'_retrieve cards))
    (clear-table)
    t))

;; Main Loop
(send *ri* :set-impedance-controller-param :arms :force-gain #f(0.4 0.4 0.4))
(format t "Reseting Pose ...~%")
(reset-pose 5000 :wait t)
(send *ri* :stop-grasp :larm)
(send deck :locate (send (send *hironxjsk* :larm-end-coords) :worldpos))
(send (send *hironxjsk* :larm-end-coords) :assoc deck)
(when *enable-viewer* (irt))

(format t ";; (pick&place '(80 40) '(-10 40) '(-100 40 t))~%")
(format t ";; (retrieve-all-cards)~%")
