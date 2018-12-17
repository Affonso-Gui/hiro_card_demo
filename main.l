(require :hiro-interface "package://hrpsys_ros_bridge_tutorials/euslisp/hironxjsk-interface.l")
(load "models/room73b2-foldable-desk-object.l")
(load "models/arrow-object.l")

(defmethod HIRONXJSK-robot
  (:reset-pose ()
    (send self :angle-vector #f(-7.28546 -4.9056 -103.32 -77.7793 11.3808 -16.043 13.4646 1.82886 -88.6255 119.881 13.3115 3.92436 0.0 2.135382e-18 29.9943))))

(hironxjsk-init)
(setq table (room73b2-foldable-desk))
(send table :locate #f(400 0.0 -800.0) :world)

(objects (list *hironxjsk* table))

;; RHAND-END-COORDS
(setq rhand-end-coords (send (arrow) :newcoords (send (send *hironxjsk* :rarm-end-coords) :copy-worldcoords)))
(send rhand-end-coords :translate #f(0 0 -70))
(send (send *hironxjsk* :rarm-end-coords) :assoc rhand-end-coords)

;; Utility function
(defun irt () (send *irtviewer* :draw-objects))

(defun waist-transform (coords)
  (send (send (send *hironxjsk* :torso-end-coords) :copy-worldcoords)
        :transform coords))

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
        (v+ #f(-10 -10 0 0) (send *ri* :hand-angle-vector :lhand)) 750)
  (send *ri* :hand-effort :lhand '(100 100 nil nil))
  (unix:sleep 1)

  (send *ri* :hand-effort :lhand '(nil nil nil 1))
  (send *ri* :hand-angle-vector :lhand #f(-55 -11.8 -25 30) 800) ;; push card
  (unix:usleep (* 750 1000))

  (send *ri* :hand-effort :lhand 3))

(defun pick-card ()
  (send *ri* :hand-width :rhand 20)
  (send-rarm #f(6.46764 0.99122 -105.688 -73.8493 -2.26606 -15.871))
  (send-robot 1000 :wait t)

  (send *ri* :hand-width :rhand -2 :time 500)
  (unix:sleep 1)

  (send *ri* :stop-grasp :larm)
  ;;(reset-pose 1000 :wait t)
  )

(defun place-card (x y &optional flip)
  (if flip (setq x (- x 63)))

  (let ((v1 (send-rarm #f(-14.7001 -21.3791 -86.619 -79.6632 18.2476 -14.3879)))
        (v2 (send *hironxjsk* :rarm
                  :inverse-kinematics (waist-transform (make-coords :pos (float-vector x y 6.151) :rpy #f(0.112 0.029 1.005)))
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
                                                                    :rpy #f(0.0 -0.023 0.261)))
                  :move-target rhand-end-coords))
        (v5 (send *hironxjsk* :rarm :move-end-pos #f(0 0 30))))

    (when flip
      (setf (elt v1 5) -40 ;; rarm-wrist-r
            (elt v2 5) -163
            (elt v3 5) -163
            (elt v4 5) -163
            (elt v5 5) -163))

    ;; place on table
    (send *ri* :angle-vector-sequence (list v1 v2 v3 v4) (list 750 750 1000 750))
    (unix:usleep (* 2500 1000))
    (if flip
      (send *ri* :hand-angle-vector :rhand #f(-20.5986 26.4 -25 90.5) 750)
      (send *ri* :hand-angle-vector :rhand #f(25 -90.5 20.5986 -26.4) 750))
    (send *ri* :wait-interpolation)
    (unix:usleep (* 250 1000))

    ;; go back
    (send *ri* :hand-width :rhand 30 :time 2000)
    (send *ri* :angle-vector-sequence (list v5 (send *hironxjsk* :reset-pose)) (list 750 1500))))

(defun pick&place (&rest cards)
  (dolist (card cards)
      (bench (push-card))
      (bench (pick-card))
      (bench (apply #'place-card card))))
