(provide :two-dof-manipulator)


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; two-dof-manipulator
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defclass two-dof-manipulator
  :super robot-model
  :slots (joint1
          joint2
          end-coords
          )
  )

(defmethod two-dof-manipulator
  (:init
   (&key
    (name "two-dof-manipulator")
    (length1 1000.0)
    (length2 1000.0)
    (width 200.0)
    (thickness 10.0)
    )
   (send-super :init :name name)
   (let* ((link0
           (instance bodyset-link :init (make-cascoords) :name "link0"
                     :bodies
                     (list (send self :_generate-body
                                 :length 100 :width width :thickness thickness :unilateral-shape? t))))
          (link1
           (instance bodyset-link :init (make-cascoords) :name "link1"
                     :bodies
                     (list (send self :_generate-body
                                 :length length1 :width width :thickness (* 0.9 thickness)))))
          (link2
           (instance bodyset-link :init (make-cascoords) :name "link2"
                     :bodies
                     (list (send self :_generate-body
                                 :length length2 :width width :thickness (* 0.8 thickness) :end-coords? t))))
          )
     (setq end-coords (make-cascoords :parent link2 :name :end-coords :pos (float-vector length2 0 0)))
     (setq links (list link0 link1 link2))
     ;; color
     (send-all (send link0 :bodies) :set-color #f(0 0 1))
     (send-all (send link1 :bodies) :set-color #f(1 0 0))
     (send-all (send link2 :bodies) :set-color #f(0 1 0))
     ;; assoc
     (send link0 :rotate pi :z)
     (send link2 :translate (float-vector length1 0 0))
     (send link1 :assoc link2)
     (send link0 :assoc link1)
     (send self :assoc link0)
     ;; joint
     (setq joint1
           (instance rotational-joint :init :parent-link link0 :child-link link1
                     :name (send link1 :name) :axis :z :min *-inf* :max *inf*))
     (setq joint2
           (instance rotational-joint :init :parent-link link1 :child-link link2
                     :name (send link2 :name) :axis :z :min *-inf* :max *inf*))
     (setq joint-list (list joint1 joint2))
     ;; end
     (send self :init-ending)
     self
     ))
  (:end-coords
   ()
   end-coords
   )
  (:_generate-body
    (&key
     (length 100.0)
     (width 50.0)
     (thickness 10.0)
     (unilateral-shape? nil)
     (end-coords? nil)
     )
    (let* ((bottom-point-list)
           (semicircle-point-list-1)
           (semicircle-point-list-2)
           (semicircle-divide-num 10)
           )
      (cond (end-coords?
             (push
              (float-vector length 0 0)
              semicircle-point-list-1)
             )
            ((not unilateral-shape?)
             (dotimes (i semicircle-divide-num)
               (let* ((angle
                       (+ (* pi (/ (float (1+ i)) (1+ semicircle-divide-num))) -pi/2))
                      )
                 (push
                  (float-vector (+ length (* 0.5 width (cos angle))) (* 0.5 width (sin angle)) 0)
                  semicircle-point-list-1)
                 ))
             ))
      (setq semicircle-point-list-1 (reverse semicircle-point-list-1))
      (dotimes (i semicircle-divide-num)
        (let* ((angle
                (+ (* pi (/ (float (1+ i)) (1+ semicircle-divide-num))) pi/2))
               )
          (push
           (float-vector (* 0.5 width (cos angle)) (* 0.5 width (sin angle)) 0)
           semicircle-point-list-2)
          ))
      (setq semicircle-point-list-2 (reverse semicircle-point-list-2))
      (setq bottom-point-list
            (append
             (list
              (float-vector 0 (* -0.5 width) 0)
              (float-vector (if end-coords? (- length width) length) (* -0.5 width) 0))
             semicircle-point-list-1
             (list
              (float-vector (if end-coords? (- length width) length) (* 0.5 width) 0)
              (float-vector 0 (* 0.5 width) 0))
             semicircle-point-list-2
             ))
      (make-prism bottom-point-list thickness)
      ))
  )
