(load "package://eus_qp/optmotiongen/euslisp/trajectory-configuration-task.l")
(load "package://eus_qp/optmotiongen/euslisp/inverse-kinematics-statics-wrapper.l")
(load "package://eus_qp/optmotiongen/euslisp/sqp-optimization.l")
(load "package://eus_qp/optmotiongen/euslisp/validate-jacobian.l")
(require :rhp3 "package://khi_ros_bridge_tutorials/euslisp/rhp3-interface.l")
(require "models/arrow-object.l")


(defun setup-rhp3-for-demo-rhp3-wholebody-contact
    ()
  (setq *robot* (instance rhp3-robot :init))
  ;; assoc contact coords
  ;;;; foot
  (send *robot* :put :rleg-foot-contact-coords
        (make-cascoords
         :parent
         (send (send *robot* :rleg :end-coords) :parent)
         :coords
         (send (send *robot* :rleg :end-coords) :copy-worldcoords)))
  (send *robot* :put :lleg-foot-contact-coords
        (make-cascoords
         :parent
         (send (send *robot* :lleg :end-coords) :parent)
         :coords
         (send (send *robot* :lleg :end-coords) :copy-worldcoords)))
  ;;;; hand
  (send *robot* :put :rarm-hand-contact-coords
        (make-cascoords
         :parent
         (send (send *robot* :rarm :end-coords) :parent)
         :coords
         (send (send (send *robot* :rarm :end-coords) :copy-worldcoords)
               :transform
               (make-coords :pos #f(30 0 0) :rpy (list -pi/2 0 pi/2)))))
  (send *robot* :put :larm-hand-contact-coords
        (make-cascoords
         :parent
         (send (send *robot* :larm :end-coords) :parent)
         :coords
         (send (send (send *robot* :larm :end-coords) :copy-worldcoords)
               :transform
               (make-coords :pos #f(30 0 0) :rpy (list pi/2 0 -pi/2)))))
  ;;;; knee
  (send *robot* :put :lleg-knee-contact-coords
        (make-cascoords
         :parent (send (send *robot* :lleg :knee-p) :child-link)
         :coords
         (send (send (send (send *robot* :lleg :knee-p) :child-link) :copy-worldcoords)
               :transform (make-coords :pos #f(110 0 0) :rpy (list 0 -pi/2 0)))))
  (send *robot* :put :rleg-knee-contact-coords
        (make-cascoords
         :parent (send (send *robot* :rleg :knee-p) :child-link)
         :coords
         (send (send (send (send *robot* :rleg :knee-p) :child-link) :copy-worldcoords)
               :transform (make-coords :pos #f(110 0 0) :rpy (list 0 -pi/2 0)))))
  ;;;; elbow
  (send *robot* :put :rarm-elbow-contact-coords
        (make-cascoords
         :parent
         (send *robot* :rarm :elbow-p :child-link)
         :coords
         (send
          (send (send *robot* :rarm :elbow-p :child-link) :copy-worldcoords)
          :transform (make-coords :pos #f(-75 -40 0) :rpy (list 0 pi/2 0)))
         ))
  (send *robot* :put :larm-elbow-contact-coords
        (make-cascoords
         :parent
         (send *robot* :larm :elbow-p :child-link)
         :coords
         (send
          (send (send *robot* :larm :elbow-p :child-link) :copy-worldcoords)
          :transform (make-coords :pos #f(-75 40 0) :rpy (list 0 pi/2 0)))
         ))
  ;; save pose
  (send *robot* :reset-pose)
  (send *robot* :put :reset-pose-joint-angle-list (coerce (send *robot* :angle-vector) cons))
  (send *robot* :move-coords
        (make-coords)
        (midcoords 0.5
                   (send *robot* :get :rleg-foot-contact-coords)
                   (send *robot* :get :lleg-foot-contact-coords)))
  )

(defun setup-contact-constraint-for-demo-rhp3-wholebody-contact
    ()
  (send *robot* :put :lleg-foot-contact-constraint
        (instance default-contact-constraint :init
                  :mu-trans 0.1 :mu-rot 0.01 :max-fz 1000
                  :l-min-x -1.0 :l-max-x 1.0 :l-min-y -1.0 :l-max-y 1.0
                  ))
  (send *robot* :put :rleg-foot-contact-constraint
        (instance default-contact-constraint :init
                  :mu-trans 0.1 :mu-rot 0.01 :max-fz 1000
                  :l-min-x -1.0 :l-max-x 1.0 :l-min-y -1.0 :l-max-y 1.0
                  ))
  (send *robot* :put :lleg-knee-contact-constraint
        (instance default-contact-constraint :init
                  :mu-trans 0.1 :mu-rot 0.01 :max-fz 1000
                  :l-min-x -1.0 :l-max-x 1.0 :l-min-y -1.0 :l-max-y 1.0
                  ))
  (send *robot* :put :rleg-knee-contact-constraint
        (instance default-contact-constraint :init
                  :mu-trans 0.1 :mu-rot 0.01 :max-fz 1000
                  :l-min-x -1.0 :l-max-x 1.0 :l-min-y -1.0 :l-max-y 1.0
                  ))
  (send *robot* :put :larm-hand-contact-constraint
        (instance default-contact-constraint :init
                  :mu-trans 0.1 :mu-rot 0.01 :max-fz 1000
                  :l-min-x -1.0 :l-max-x 1.0 :l-min-y -1.0 :l-max-y 1.0
                  ))
  (send *robot* :put :rarm-hand-contact-constraint
        (instance default-contact-constraint :init
                  :mu-trans 0.1 :mu-rot 0.01 :max-fz 1000
                  :l-min-x -1.0 :l-max-x 1.0 :l-min-y -1.0 :l-max-y 1.0
                  ))
  )

(defun demo-rhp3-wholebody-contact-trajectory
    (&key
     (optimize-torque? nil)
     (optimize-start-end-torque? t)
     (output-filename
      (ros::resolve-ros-path "package://eus_qp/optmotiongen/logs/demo-rhp3-wholebody-contact-trajectory/tmp"))
     (pre-process-func)
     (visualize-callback-func)
     (visualize-loop? t)
     &allow-other-keys
     )
  ;; setup robot
  (setup-rhp3-for-demo-rhp3-wholebody-contact)
  (setq *contact-list*
        (list (instance contact-ik-arg :init
                        :name :lleg-knee
                        :target-coords (make-coords)
                        :translation-axis (list :x :y)
                        :rotation-axis t
                        :transform-coords (make-coords))
              (instance contact-ik-arg :init
                        :name :rleg-knee
                        :target-coords (make-coords)
                        :translation-axis (list :x :y)
                        :rotation-axis t
                        :transform-coords (make-coords))
              (instance contact-ik-arg :init
                        :name :larm-hand
                        :target-coords (make-coords)
                        :translation-axis (list :x :y)
                        :rotation-axis t
                        :transform-coords (make-coords))
              (instance contact-ik-arg :init
                        :name :rarm-hand
                        :target-coords (make-coords)
                        :translation-axis (list :x :y)
                        :rotation-axis t
                        :transform-coords (make-coords))
              ))
  (setq *robot-env*
        (instance robot-environment :init :robot *robot* :contact-list *contact-list*))
  (send (send (send *robot-env* :contact :larm-hand) :joint :larm-hand-linear-joint-x)
        :min-angle 400.0)
  (send (send (send *robot-env* :contact :rarm-hand) :joint :rarm-hand-linear-joint-x)
        :min-angle 400.0)
  (send (send *robot-env* :contact :larm-hand) :put :pre-contact-coords
        (make-cascoords
         :parent
         (send *robot-env* :contact :larm-hand :contact-coords)
         :coords
         (send (send (send *robot-env* :contact :larm-hand :contact-coords) :copy-worldcoords)
               :translate (float-vector 0 0 150))
         ))
  (send (send *robot-env* :contact :rarm-hand) :put :pre-contact-coords
        (make-cascoords
         :parent
         (send *robot-env* :contact :rarm-hand :contact-coords)
         :coords
         (send (send (send *robot-env* :contact :rarm-hand :contact-coords) :copy-worldcoords)
               :translate (float-vector 0 0 150))
         ))
  (setup-contact-constraint-for-demo-rhp3-wholebody-contact)
  (setq *robot-env-list* nil)
  (setq *instant-config-task-list* nil)
  ;; draw
  (unless (boundp '*irtviewer*)
    (make-irtviewer)
    (send *irtviewer* :change-background (float-vector 1 1 1))
    (send *irtviewer* :draw-floor 500)
    (send *irtviewer* :floor-color (float-vector 0 0 0))
    )

  (push (copy-object *robot-env*) *robot-env-list*)
  (push
   (instance instant-configuration-task :init
             :name :knee-stand
             :robot-env (car *robot-env-list*)
             :drive-joint-list
             nil ;; (send (car *robot-env-list*) :drive-joint-list)
             :kin-target-coords-list
             (list
              (send (send (car *robot-env-list*) :robot :get :lleg-foot-contact-coords) :copy-worldcoords)
              (send (send (car *robot-env-list*) :robot :get :rleg-foot-contact-coords) :copy-worldcoords)
              )
             :kin-attention-coords-list
             (list
              (send (car *robot-env-list*) :robot :get :lleg-foot-contact-coords)
              (send (car *robot-env-list*) :robot :get :rleg-foot-contact-coords)
              )
             :contact-target-coords-list
             (list
              (send (send (car *robot-env-list*) :robot :get :lleg-foot-contact-coords) :copy-worldcoords)
              (send (send (car *robot-env-list*) :robot :get :rleg-foot-contact-coords) :copy-worldcoords)
              )
             :contact-attention-coords-list
             (list
              (send (car *robot-env-list*) :robot :get :lleg-foot-contact-coords)
              (send (car *robot-env-list*) :robot :get :rleg-foot-contact-coords)
              )
             :contact-constraint-list
             (list
              (send (car *robot-env-list*) :robot :get :lleg-foot-contact-constraint)
              (send (car *robot-env-list*) :robot :get :rleg-foot-contact-constraint)
              )
             :posture-joint-list
             (send (car *robot-env-list*) :robot :joint-list)
             :posture-joint-angle-list
             (send (car *robot-env-list*) :robot :get :reset-pose-joint-angle-list)
             :target-posture-scale 1e-1
             )
   *instant-config-task-list*
   )

  (dotimes (i 3)
    (push (copy-object *robot-env*) *robot-env-list*)
    (push
     (instance instant-configuration-task :init
               :name :knee-stand
               :robot-env (car *robot-env-list*)
               :drive-joint-list
               nil ;; (send (car *robot-env-list*) :drive-joint-list)
               :kin-target-coords-list
               (list
                (send (send (car *robot-env-list*) :robot :get :lleg-foot-contact-coords) :copy-worldcoords)
                (send (send (car *robot-env-list*) :robot :get :rleg-foot-contact-coords) :copy-worldcoords)
                )
               :kin-attention-coords-list
               (list
                (send (car *robot-env-list*) :robot :get :lleg-foot-contact-coords)
                (send (car *robot-env-list*) :robot :get :rleg-foot-contact-coords)
                )
               :contact-target-coords-list
               (list
                (send (send (car *robot-env-list*) :robot :get :lleg-foot-contact-coords) :copy-worldcoords)
                (send (send (car *robot-env-list*) :robot :get :rleg-foot-contact-coords) :copy-worldcoords)
                )
               :contact-attention-coords-list
               (list
                (send (car *robot-env-list*) :robot :get :lleg-foot-contact-coords)
                (send (car *robot-env-list*) :robot :get :rleg-foot-contact-coords)
                )
               :contact-constraint-list
               (list
                (send (car *robot-env-list*) :robot :get :lleg-foot-contact-constraint)
                (send (car *robot-env-list*) :robot :get :rleg-foot-contact-constraint)
                )
               )
     *instant-config-task-list*
     )
    )

  (push (copy-object *robot-env*) *robot-env-list*)
  (push
   (instance instant-configuration-task :init
             :name :knee-stand
             :robot-env (car *robot-env-list*)
             :drive-joint-list
             nil ;; (send (car *robot-env-list*) :drive-joint-list)
             :kin-target-coords-list
             (list
              (send (send (car *robot-env-list*) :robot :get :lleg-foot-contact-coords) :copy-worldcoords)
              (send (send (car *robot-env-list*) :robot :get :rleg-foot-contact-coords) :copy-worldcoords)
              )
             :kin-attention-coords-list
             (list
              (send (car *robot-env-list*) :robot :get :lleg-foot-contact-coords)
              (send (car *robot-env-list*) :robot :get :rleg-foot-contact-coords)
              )
             :contact-target-coords-list
             (list
              (send (send (car *robot-env-list*) :robot :get :lleg-foot-contact-coords) :copy-worldcoords)
              (send (send (car *robot-env-list*) :robot :get :rleg-foot-contact-coords) :copy-worldcoords)
              )
             :contact-attention-coords-list
             (list
              (send (car *robot-env-list*) :robot :get :lleg-foot-contact-coords)
              (send (car *robot-env-list*) :robot :get :rleg-foot-contact-coords)
              )
             :contact-constraint-list
             (list
              (send (car *robot-env-list*) :robot :get :lleg-foot-contact-constraint)
              (send (car *robot-env-list*) :robot :get :rleg-foot-contact-constraint)
              )
             )
   *instant-config-task-list*
   )

  (push (copy-object *robot-env*) *robot-env-list*)
  (push
   (instance instant-configuration-task :init
             :name :knee-stand
             :robot-env (car *robot-env-list*)
             :drive-joint-list
             nil ;; (send (car *robot-env-list*) :drive-joint-list)
             :kin-target-coords-list
             (list
              (send (send (car *robot-env-list*) :robot :get :lleg-foot-contact-coords) :copy-worldcoords)
              (send (send (car *robot-env-list*) :robot :get :rleg-foot-contact-coords) :copy-worldcoords)
              (send (send (car *robot-env-list*) :contact :larm-hand) :get :pre-contact-coords)
              (send (send (car *robot-env-list*) :contact :rarm-hand) :get :pre-contact-coords)
              )
             :kin-attention-coords-list
             (list
              (send (car *robot-env-list*) :robot :get :lleg-foot-contact-coords)
              (send (car *robot-env-list*) :robot :get :rleg-foot-contact-coords)
              (send (car *robot-env-list*) :robot :get :larm-hand-contact-coords)
              (send (car *robot-env-list*) :robot :get :rarm-hand-contact-coords)
              )
             :contact-target-coords-list
             (list
              (send (send (car *robot-env-list*) :robot :get :lleg-foot-contact-coords) :copy-worldcoords)
              (send (send (car *robot-env-list*) :robot :get :rleg-foot-contact-coords) :copy-worldcoords)
              )
             :contact-attention-coords-list
             (list
              (send (car *robot-env-list*) :robot :get :lleg-foot-contact-coords)
              (send (car *robot-env-list*) :robot :get :rleg-foot-contact-coords)
              )
             :contact-constraint-list
             (list
              (send (car *robot-env-list*) :robot :get :lleg-foot-contact-constraint)
              (send (car *robot-env-list*) :robot :get :rleg-foot-contact-constraint)
              )
             )
   *instant-config-task-list*
   )

  (push (copy-object *robot-env*) *robot-env-list*)
  (push
   (instance instant-configuration-task :init
             :name :knee-stand
             :robot-env (car *robot-env-list*)
             :drive-joint-list
             nil ;; (send (car *robot-env-list*) :drive-joint-list)
             :kin-target-coords-list
             (list
              (send (send (car *robot-env-list*) :robot :get :lleg-foot-contact-coords) :copy-worldcoords)
              (send (send (car *robot-env-list*) :robot :get :rleg-foot-contact-coords) :copy-worldcoords)
              (send (send (car *robot-env-list*) :contact :larm-hand) :contact-coords)
              (send (send (car *robot-env-list*) :contact :rarm-hand) :contact-coords)
              )
             :kin-attention-coords-list
             (list
              (send (car *robot-env-list*) :robot :get :lleg-foot-contact-coords)
              (send (car *robot-env-list*) :robot :get :rleg-foot-contact-coords)
              (send (car *robot-env-list*) :robot :get :larm-hand-contact-coords)
              (send (car *robot-env-list*) :robot :get :rarm-hand-contact-coords)
              )
             :contact-target-coords-list
             (list
              (send (send (car *robot-env-list*) :robot :get :lleg-foot-contact-coords) :copy-worldcoords)
              (send (send (car *robot-env-list*) :robot :get :rleg-foot-contact-coords) :copy-worldcoords)
              )
             :contact-attention-coords-list
             (list
              (send (car *robot-env-list*) :robot :get :lleg-foot-contact-coords)
              (send (car *robot-env-list*) :robot :get :rleg-foot-contact-coords)
              )
             :contact-constraint-list
             (list
              (send (car *robot-env-list*) :robot :get :lleg-foot-contact-constraint)
              (send (car *robot-env-list*) :robot :get :rleg-foot-contact-constraint)
              )
             )
   *instant-config-task-list*
   )

  (push (copy-object *robot-env*) *robot-env-list*)
  (push
   (instance instant-configuration-task :init
             :name :knee-stand
             :robot-env (car *robot-env-list*)
             :drive-joint-list
             nil ;; (send (car *robot-env-list*) :drive-joint-list)
             :kin-target-coords-list
             (list
              (send (send (car *robot-env-list*) :robot :get :lleg-foot-contact-coords) :copy-worldcoords)
              (send (send (car *robot-env-list*) :robot :get :rleg-foot-contact-coords) :copy-worldcoords)
              (send (send (car *robot-env-list*) :contact :larm-hand) :contact-coords)
              (send (send (car *robot-env-list*) :contact :rarm-hand) :contact-coords)
              )
             :kin-attention-coords-list
             (list
              (send (car *robot-env-list*) :robot :get :lleg-foot-contact-coords)
              (send (car *robot-env-list*) :robot :get :rleg-foot-contact-coords)
              (send (car *robot-env-list*) :robot :get :larm-hand-contact-coords)
              (send (car *robot-env-list*) :robot :get :rarm-hand-contact-coords)
              )
             :contact-target-coords-list
             (list
              (send (send (car *robot-env-list*) :robot :get :lleg-foot-contact-coords) :copy-worldcoords)
              (send (send (car *robot-env-list*) :robot :get :rleg-foot-contact-coords) :copy-worldcoords)
              (send (send (car *robot-env-list*) :contact :larm-hand) :contact-coords)
              (send (send (car *robot-env-list*) :contact :rarm-hand) :contact-coords)
              )
             :contact-attention-coords-list
             (list
              (send (car *robot-env-list*) :robot :get :lleg-foot-contact-coords)
              (send (car *robot-env-list*) :robot :get :rleg-foot-contact-coords)
              (send (car *robot-env-list*) :robot :get :larm-hand-contact-coords)
              (send (car *robot-env-list*) :robot :get :rarm-hand-contact-coords)
              )
             :contact-constraint-list
             (list
              (send (car *robot-env-list*) :robot :get :lleg-foot-contact-constraint)
              (send (car *robot-env-list*) :robot :get :rleg-foot-contact-constraint)
              (send (car *robot-env-list*) :robot :get :larm-hand-contact-constraint)
              (send (car *robot-env-list*) :robot :get :rarm-hand-contact-constraint)
              )
             )
   *instant-config-task-list*
   )

  (push (copy-object *robot-env*) *robot-env-list*)
  (push
   (instance instant-configuration-task :init
             :name :knee-stand
             :robot-env (car *robot-env-list*)
             :drive-joint-list
             nil ;; (send (car *robot-env-list*) :drive-joint-list)
             :kin-target-coords-list
             (list
              (send (send (car *robot-env-list*) :robot :get :lleg-foot-contact-coords) :copy-worldcoords)
              (send (send (car *robot-env-list*) :robot :get :rleg-foot-contact-coords) :copy-worldcoords)
              (send (send (car *robot-env-list*) :contact :larm-hand) :contact-coords)
              (send (send (car *robot-env-list*) :contact :rarm-hand) :contact-coords)
              )
             :kin-attention-coords-list
             (list
              (send (car *robot-env-list*) :robot :get :lleg-foot-contact-coords)
              (send (car *robot-env-list*) :robot :get :rleg-foot-contact-coords)
              (send (car *robot-env-list*) :robot :get :larm-hand-contact-coords)
              (send (car *robot-env-list*) :robot :get :rarm-hand-contact-coords)
              )
             :contact-target-coords-list
             (list
              (send (send (car *robot-env-list*) :robot :get :lleg-foot-contact-coords) :copy-worldcoords)
              (send (send (car *robot-env-list*) :robot :get :rleg-foot-contact-coords) :copy-worldcoords)
              (send (send (car *robot-env-list*) :contact :larm-hand) :contact-coords)
              (send (send (car *robot-env-list*) :contact :rarm-hand) :contact-coords)
              )
             :contact-attention-coords-list
             (list
              (send (car *robot-env-list*) :robot :get :lleg-foot-contact-coords)
              (send (car *robot-env-list*) :robot :get :rleg-foot-contact-coords)
              (send (car *robot-env-list*) :robot :get :larm-hand-contact-coords)
              (send (car *robot-env-list*) :robot :get :rarm-hand-contact-coords)
              )
             :contact-constraint-list
             (list
              (send (car *robot-env-list*) :robot :get :lleg-foot-contact-constraint)
              (send (car *robot-env-list*) :robot :get :rleg-foot-contact-constraint)
              (send (car *robot-env-list*) :robot :get :larm-hand-contact-constraint)
              (send (car *robot-env-list*) :robot :get :rarm-hand-contact-constraint)
              )
             )
   *instant-config-task-list*
   )

  (push (copy-object *robot-env*) *robot-env-list*)
  (push
   (instance instant-configuration-task :init
             :name :knee-stand
             :robot-env (car *robot-env-list*)
             :drive-joint-list
             nil ;; (send (car *robot-env-list*) :drive-joint-list)
             :kin-target-coords-list
             (list
              (send (send (car *robot-env-list*) :robot :get :lleg-foot-contact-coords) :copy-worldcoords)
              (send (send (car *robot-env-list*) :robot :get :rleg-foot-contact-coords) :copy-worldcoords)
              (send (send (car *robot-env-list*) :contact :lleg-knee) :contact-coords)
              (send (send (car *robot-env-list*) :contact :rleg-knee) :contact-coords)
              (send (send (car *robot-env-list*) :contact :larm-hand) :contact-coords)
              (send (send (car *robot-env-list*) :contact :rarm-hand) :contact-coords)
              )
             :kin-attention-coords-list
             (list
              (send (car *robot-env-list*) :robot :get :lleg-foot-contact-coords)
              (send (car *robot-env-list*) :robot :get :rleg-foot-contact-coords)
              (send (car *robot-env-list*) :robot :get :lleg-knee-contact-coords)
              (send (car *robot-env-list*) :robot :get :rleg-knee-contact-coords)
              (send (car *robot-env-list*) :robot :get :larm-hand-contact-coords)
              (send (car *robot-env-list*) :robot :get :rarm-hand-contact-coords)
              )
             :contact-target-coords-list
             (list
              (send (send (car *robot-env-list*) :robot :get :lleg-foot-contact-coords) :copy-worldcoords)
              (send (send (car *robot-env-list*) :robot :get :rleg-foot-contact-coords) :copy-worldcoords)
              (send (send (car *robot-env-list*) :contact :lleg-knee) :contact-coords)
              (send (send (car *robot-env-list*) :contact :rleg-knee) :contact-coords)
              (send (send (car *robot-env-list*) :contact :larm-hand) :contact-coords)
              (send (send (car *robot-env-list*) :contact :rarm-hand) :contact-coords)
              )
             :contact-attention-coords-list
             (list
              (send (car *robot-env-list*) :robot :get :lleg-foot-contact-coords)
              (send (car *robot-env-list*) :robot :get :rleg-foot-contact-coords)
              (send (car *robot-env-list*) :robot :get :lleg-knee-contact-coords)
              (send (car *robot-env-list*) :robot :get :rleg-knee-contact-coords)
              (send (car *robot-env-list*) :robot :get :larm-hand-contact-coords)
              (send (car *robot-env-list*) :robot :get :rarm-hand-contact-coords)
              )
             :contact-constraint-list
             (list
              (send (car *robot-env-list*) :robot :get :lleg-foot-contact-constraint)
              (send (car *robot-env-list*) :robot :get :rleg-foot-contact-constraint)
              (send (car *robot-env-list*) :robot :get :lleg-knee-contact-constraint)
              (send (car *robot-env-list*) :robot :get :rleg-knee-contact-constraint)
              (send (car *robot-env-list*) :robot :get :larm-hand-contact-constraint)
              (send (car *robot-env-list*) :robot :get :rarm-hand-contact-constraint)
              )
             :posture-joint-list
             (send (car *robot-env-list*) :robot :legs :knee-p)
             :posture-joint-angle-list
             (send (car *robot-env-list*) :robot :legs :knee-p :max-angle)
             :target-posture-scale 1e3
             )
   *instant-config-task-list*
   )

  (push (copy-object *robot-env*) *robot-env-list*)
  (push
   (instance instant-configuration-task :init
             :name :knee-stand
             :robot-env (car *robot-env-list*)
             :drive-joint-list
             nil ;; (send (car *robot-env-list*) :drive-joint-list)
             :kin-target-coords-list
             (list
              (send (send (car *robot-env-list*) :robot :get :lleg-foot-contact-coords) :copy-worldcoords)
              (send (send (car *robot-env-list*) :robot :get :rleg-foot-contact-coords) :copy-worldcoords)
              (send (send (car *robot-env-list*) :contact :lleg-knee) :contact-coords)
              (send (send (car *robot-env-list*) :contact :rleg-knee) :contact-coords)
              (send (send (car *robot-env-list*) :contact :larm-hand) :get :pre-contact-coords)
              (send (send (car *robot-env-list*) :contact :rarm-hand) :get :pre-contact-coords)
              )
             :kin-attention-coords-list
             (list
              (send (car *robot-env-list*) :robot :get :lleg-foot-contact-coords)
              (send (car *robot-env-list*) :robot :get :rleg-foot-contact-coords)
              (send (car *robot-env-list*) :robot :get :lleg-knee-contact-coords)
              (send (car *robot-env-list*) :robot :get :rleg-knee-contact-coords)
              (send (car *robot-env-list*) :robot :get :larm-hand-contact-coords)
              (send (car *robot-env-list*) :robot :get :rarm-hand-contact-coords)
              )
             :contact-target-coords-list
             (list
              (send (send (car *robot-env-list*) :robot :get :lleg-foot-contact-coords) :copy-worldcoords)
              (send (send (car *robot-env-list*) :robot :get :rleg-foot-contact-coords) :copy-worldcoords)
              (send (send (car *robot-env-list*) :contact :lleg-knee) :contact-coords)
              (send (send (car *robot-env-list*) :contact :rleg-knee) :contact-coords)
              )
             :contact-attention-coords-list
             (list
              (send (car *robot-env-list*) :robot :get :lleg-foot-contact-coords)
              (send (car *robot-env-list*) :robot :get :rleg-foot-contact-coords)
              (send (car *robot-env-list*) :robot :get :lleg-knee-contact-coords)
              (send (car *robot-env-list*) :robot :get :rleg-knee-contact-coords)
              )
             :contact-constraint-list
             (list
              (send (car *robot-env-list*) :robot :get :lleg-foot-contact-constraint)
              (send (car *robot-env-list*) :robot :get :rleg-foot-contact-constraint)
              (send (car *robot-env-list*) :robot :get :lleg-knee-contact-constraint)
              (send (car *robot-env-list*) :robot :get :rleg-knee-contact-constraint)
              )
             :posture-joint-list
             (send (car *robot-env-list*) :robot :legs :knee-p)
             :posture-joint-angle-list
             (send (car *robot-env-list*) :robot :legs :knee-p :max-angle)
             :target-posture-scale 1e3
             )
   *instant-config-task-list*
   )

  ;; set color
  (dotimes (i (length *robot-env-list*))
    (send-all (send (elt *robot-env-list* i) :bodies) :set-color
              (map float-vector #'(lambda (x) (/ x 255.0)) (color-category20 i)))
    )
  (objects *robot-env-list*)

  (setq *trajectory-config-task*
        (instance trajectory-configuration-task :init
                  :instant-config-task-list (reverse *instant-config-task-list*)
                  ))

  (setq *sqp-opt*
        (instance sqp-optimization :init :config-task *trajectory-config-task*))
  (send *sqp-opt* :optimize :loop-num 30)

  ;; save log
  (when (and (not (equal (send *sqp-opt* :status) :qp-error)) output-filename)
    (setq *rs-list*
          (send *trajectory-config-task* :generate-robot-state-list
                :robot-env *robot-env*
                :joint-name-list (send-all (send *robot-env* :robot :joint-list) :name)
                :root-link-name (send (car (send *robot-env* :robot :links)) :name)
                :step-time 1.0
                :divide-num 1
                ))
    (objects (list *robot-env*))
    (play-robot-state-list
     :rs-list *rs-list*
     :robot-env *robot-env*
     :joint-name-list (send-all (send *robot-env* :robot :joint-list) :name)
     :root-link-name (send (car (send *robot-env* :robot :links)) :name)
     )
    (dump-loadable-structure (format nil "~a.l" output-filename) *rs-list*)
    )

  (objects (list *robot-env*))
  (when (not (equal (send *sqp-opt* :status) :qp-error))
    (send *trajectory-config-task* :play-animation
          :robot-env *robot-env*
          :loop? t
          :visualize-force? t)
    )
  )
(warn "(demo-rhp3-wholebody-contact-trajectory)~%")
