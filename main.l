(require :hiro-interface "package://hrpsys_ros_bridge_tutorials/euslisp/hironxjsk-interface.l")
(load "models/room73b2-foldable-desk-object.l")
(load "models/arrow-object.l")

(defmethod HIRONXJSK-robot
  (:reset-pose ()
    (send self :angle-vector #f(6.30106 -2.04927 -112.055 -73.767 0.513765 -24.8619 19.2455 -1.896 -95.7045 122.931 12.6211 16.4734 89.9981 0.0 29.9943))))

(hironxjsk-init)
(setq table (room73b2-foldable-desk))
(send table :rotate pi/2 :z)
(send table :locate #f(-80.0 550.0 -800.0) :world)

(objects (list *hironxjsk* table))

;; RHAND-END-COORDS
(setq rhand-end-coords (send (arrow) :newcoords (send (send *hironxjsk* :rarm-end-coords) :copy-worldcoords)))
(send rhand-end-coords :translate #f(0 0 -70))
(send (send *hironxjsk* :rarm-end-coords) :assoc rhand-end-coords)

;; Utility function
(defun irt () (send *irtviewer* :draw-objects))

(defun send-robot (time &key wait)
  (prog1
      (send *ri* :angle-vector (send *hironxjsk* :angle-vector) time)
    (when wait (send *ri* :wait-interpolation))))

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
    (send-robot time :wait wait)
    (send *irtviewer* :draw-objects)))

;; Pick & Place
(defun push-card ()
  (send *ri* :hand-effort :lhand #f(3 3 50 3))
  (send *ri* :hand-angle-vector :lhand #f(-38.9986 -90.4 -45 92.2) 750) ;; grasp deck
  (unix:usleep (* 750 1000))

  (send *ri* :hand-angle-vector :lhand
        (v+ #f(-10 -10 0 0) (send *ri* :hand-angle-vector :lhand)) 500)
  (send *ri* :hand-effort :lhand '(100 100 nil nil))
  (unix:usleep (* 500 1000))

  (send *ri* :hand-effort :lhand '(nil nil nil 1))
  (send *ri* :hand-angle-vector :lhand #f(-55 -11.8 -25 30) 800) ;; push card
  (unix:usleep (* 750 1000))

  (send *ri* :hand-effort :lhand 3))

(defun pick-card ()
  (send *ri* :hand-width :rhand 20)
  (send-rarm #f(20.2752 1.1214 -111.221 -68.4368 -13.0237 -25.2876))
  (send-robot 1000 :wait t)

  (send *ri* :hand-width :rhand -2 :time 500)
  (unix:sleep 1)

  (send *ri* :stop-grasp :larm)
  ;;(reset-pose 1000 :wait t)
  )

(defun place-card (x y)
  (let ((v0 (send *hironxjsk* :reset-pose))
        (v1 (send-rarm #f(5.2364 16.9588 -129.8 3.33475 -15.0175 -64.9964)))
        (v2 (send *hironxjsk* :rarm :inverse-kinematics (make-coords :pos (float-vector x y 6.151) :rpy #f(1.683 0.029 1.005)) :move-target rhand-end-coords))
        (v3 (send *hironxjsk* :rarm :inverse-kinematics (make-coords :pos (float-vector x y -93.85) :rpy #f(1.760 -0.011 0.482)) :move-target rhand-end-coords))
        (v4 (send *hironxjsk* :rarm :inverse-kinematics (make-coords :pos (float-vector (+ x 14.636) (+ y 7.742) -117.777) :rpy (float-vector pi/2 -0.023 0.261)) :move-target rhand-end-coords)))

    ;; place on table
    (send *ri* :angle-vector-sequence (list v0 v1 v2 v3 v4) (list 750 750 1000 1000 750))
    (unix:usleep (* 3500 1000))
    (send *ri* :hand-angle-vector :rhand #f(25 -90.5 20.5986 -26.4) 750)
    (send *ri* :wait-interpolation)
    (unix:usleep (* 250 1000))

    ;; go back
    (send *ri* :hand-width :rhand 30 :time 2000)
    (send *ri* :angle-vector-sequence (list v1 v0) (list 1250 750))))

(defun pick&place (cards)
  (dolist (card cards)
    (multiple-value-bind (x y) card
      (bench (push-card))
      (bench (pick-card))
      (bench (place-card x y)))))

