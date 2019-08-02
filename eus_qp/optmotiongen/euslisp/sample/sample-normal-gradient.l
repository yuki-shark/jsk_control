(load "../extended-joint-link.l")
(load "../validate-jacobian.l")
(require "irteus/demo/sample-arm-model.l")


(defclass normal-validate-config-task
  :super propertied-object
  :slots (_robot
          _joint-list
          _normal-coords-list
          )
  )

(defmethod normal-validate-config-task
  (:init
   (&key
    (robot)
    (joint-list)
    (normal-coords-list)
    )
   (setq _robot robot)
   (setq _joint-list joint-list)
   (setq _normal-coords-list normal-coords-list)
   )
  ;; config
  (:theta
   ()
   (mapcar
    #'(lambda (j) (send j :angle-to-speed (send j :joint-angle ))) ;; [m] [rad]
    _joint-list)
   )
  (:set-theta
   (theta)
   (map
    nil
    #'(lambda (j a) (send j :joint-angle (send j :speed-to-angle a))) ;; [mm] [deg]
    _joint-list
    theta)
   )
  ;; normal
  (:get-normal
   ()
   (apply
    #'concatenate
    float-vector
    (send-all _normal-coords-list :rotate-vector (float-vector 0 0 1))
    )
   )
  (:get-normal-jacobian
   ()
   (send _robot :calc-normal-jacobian-from-joint-list
         :union-joint-list _joint-list
         :move-target _normal-coords-list
         )
   )
  )


(defun sample-normal-gradient-common
    (&key
     (robot *robot*)
     (joint-list (send robot :joint-list))
     (normal-coords-list)
     )
  (setq *config-task*
        (instance normal-validate-config-task :init
                  :robot robot
                  :joint-list joint-list
                  :normal-coords-list normal-coords-list
                  ))

  (setq *normal-jacobian-error*
        (calc-jacobian-error-with-diff
         :config-task *config-task*
         :config-method :theta
         :set-config-method :set-theta
         :task-value-method :get-normal
         :jacobian-method :get-normal-jacobian
         :delta 1e-6
         ))
  (warning-message 2 "normal-jacobian-error: ~a~%"
                   (norm *normal-jacobian-error*))

  *normal-jacobian-error*
  )

(defun sample-normal-gradient-sample-arm
    ()
  (setq *robot* (instance sarmclass :init))

  (sample-normal-gradient-common
   :robot *robot*
   :joint-list (butlast (send *robot* :joint-list) 2)
   :normal-coords-list (list (send *robot* :end-coords))
   )

  (send *robot* :put :end-coords2
        (make-cascoords
         :parent
         (send (send *robot* :end-coords) :parent)
         :coords
         (orient-coords-to-axis (send (send *robot* :end-coords) :copy-worldcoords) (float-vector 2.0 -1.0 0.5))
         ))
  (sample-normal-gradient-common
   :robot *robot*
   :joint-list (butlast (send *robot* :joint-list) 2)
   :normal-coords-list (list (send *robot* :end-coords) (send *robot* :get :end-coords2))
   )

  (send *robot* :angle-vector #f(10.0 -20.0 30.0 -40.0 50.0 -60.0 0.0 0.0))
  (sample-normal-gradient-common
   :robot *robot*
   :joint-list (butlast (send *robot* :joint-list) 2)
   :normal-coords-list (list (send *robot* :end-coords) (send *robot* :get :end-coords2))
   )
  )
(warn "(sample-normal-gradient-sample-arm)~%")
