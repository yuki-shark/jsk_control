(load "./util.l")


;;;;;;;;;;;;;;;;;;;;;;;;;
;; 2d-planar-contact
;;;;;;;;;;;;;;;;;;;;;;;;;

(defclass 2d-planar-contact
  :super cascaded-link
  :slots ((_contact-coords "$T_{\mathit{cnt}}$")
          (_contact-pre-coords "$T_{\mathit{cnt\mathchar`-pre}}$")
          )
  :documentation
  "
二次元平面上の長方形領域での接触座標を表す仮想の関節・リンク構造．
  "
  )

(defmethod 2d-planar-contact
  (:init
   (&key
    (name "contact")
    (fix-yaw? nil)
    (contact-pre-offset 100) ;; [mm]
    )
   "
Initialize instance
   "
   (send-super :init :name name)
   (let* (rv-ll
          rv-jl
          (string-name
           (string-left-trim ":" (format nil "~a" name)))
          (joint-class-list
           (remove
            nil
            (list linear-joint linear-joint (if fix-yaw? nil rotational-joint))
            ))
          (joint-axis-list
           (remove
            nil
            (list :x :y (if fix-yaw? nil :z))
            ))
          (root-link
           (instance bodyset-link :init
                     (make-cascoords)
                     :bodies (list (make-cube 1e-3 1e-3 1e-3))
                     :name (read-from-string (format nil ":~a-root" string-name))
                     :weight 0.0
                     ))
          )
     ;; 1. setup link
     (setq rv-ll
           (append
            (list root-link)
            (mapcar
             #'(lambda (joint-class
                        joint-axis
                        )
                 (instance bodyset-link :init (make-cascoords) :bodies (list (make-cube 1e-3 1e-3 1e-3))
                           :name
                           (read-from-string
                            (format nil ":~a-~a-~a" string-name (send joint-class :name) (symbol-name joint-axis)))
                           :weight 0.0
                           ))
             joint-class-list
             joint-axis-list
             )))
     (dotimes (i (1- (length rv-ll)))
       (send (elt rv-ll i) :assoc (elt rv-ll (1+ i))))
     (send self :assoc (car rv-ll))
     (setq links rv-ll)
     ;; 2. setup joint
     (setq rv-jl
           (mapcar
            #'(lambda (joint-class
                       joint-axis
                       parent-link
                       child-link
                       )
                (instance joint-class :init
                          :name
                          (read-from-string
                           (format nil ":~a-~a-~a" string-name (send joint-class :name) (symbol-name joint-axis)))
                          :parent-link parent-link :child-link child-link
                          :axis joint-axis :min -1e10 :max 1e10
                          :max-joint-torque 0.0
                          ))
            joint-class-list
            joint-axis-list
            (butlast rv-ll)
            (cdr rv-ll)
            ))
     (setq joint-list rv-jl)
     (send self :init-ending)
     ;; 3. set contact-coords and other settings
     (setq _contact-coords
           (make-cascoords :parent (car (last rv-ll))))
     (setq _contact-pre-coords
           (make-cascoords :pos (float-vector 0 0 contact-pre-offset) :parent (car (last rv-ll))))
     self
     ))
  (:contact-coords
   (&rest args)
   "
return $T_{\mathit{cnt}} := \{ \bm{p}_{\mathit{cnt}}, \bm{R}_{\mathit{cnt}} \}$
   "
   (forward-message-to _contact-coords args)
   )
  (:contact-pre-coords
   (&rest args)
   "
return $T_{\mathit{cnt\mathchar`-pre}} := \{ \bm{p}_{\mathit{cnt\mathchar`-pre}}, \bm{R}_{\mathit{cnt\mathchar`-pre}} \}$
   "
   (forward-message-to _contact-pre-coords args)
   )
  (:set-from-face
   (&key
    (face)
    (margin 150.0) ;; [mm]
    )
   "
set coords and min/max joint angle from face.
   "
   (let* ((x-edge (elt (send face :edges) 0))
          (x-length (max 0.0 (- (send x-edge :length) (* 2 margin))))
          (x-vec (send x-edge :direction))
          (y-edge (elt (send face :edges) 1))
          (y-length (max 0.0 (- (send y-edge :length) (* 2 margin))))
          (y-vec (send y-edge :direction))
          (z-vec (send face :normal))
          (rot (transpose (matrix x-vec y-vec z-vec)))
          (pos (elt (send face :centroid) 1))
          )
     (send self :newcoords (make-coords :pos pos :rot rot))
     (send (elt joint-list 0) :max-angle (* 0.5 x-length))
     (send (elt joint-list 0) :min-angle (* -0.5 x-length))
     (send (elt joint-list 1) :max-angle (* 0.5 y-length))
     (send (elt joint-list 1) :min-angle (* -0.5 y-length))
     ))
  )


;;;;;;;;;;;;;;;;;;;;;;;;;
;; look-at-contact
;;;;;;;;;;;;;;;;;;;;;;;;;

(defclass look-at-contact
  :super cascaded-link
  :slots ((_contact-coords "$T_{\mathit{cnt}}$")
          )
  :documentation
  "
ある点を注視するためのカメラ座標を表す仮想の関節・リンク構造．
  "
  )

(defmethod look-at-contact
  (:init
   (&key
    (name "look-at")
    (target-pos (float-vector 0 0 0))
    (camera-axis :z) ;; :x, :y, :z, :-x, :-y, or :-z
    (angle-of-view 30.0) ;; [deg]
    )
   "
Initialize instance
   "
   (send-super :init :name name)
   (let* (rv-ll
          rv-jl
          (string-name
           (string-left-trim ":" (format nil "~a" (send self :name))))
          (joint-class-list
           (list rotational-joint rotational-joint
                 linear-joint
                 rotational-joint rotational-joint rotational-joint))
          (joint-axis-list
           (list :x :y
                 :z
                 :z :x :y))
          (root-link
           (instance bodyset-link :init
                     (make-cascoords)
                     :bodies (list (make-cube 1e-3 1e-3 1e-3))
                     :name (read-from-string (format nil ":~a-root" string-name))
                     :weight 0.0
                     ))
          )
     ;; 1. setup link
     (setq rv-ll
           (append
            (list root-link)
            (mapcar
             #'(lambda (joint-class
                        joint-axis
                        )
                 (instance bodyset-link :init
                           (make-cascoords)
                           :bodies (list (make-cube 1e-3 1e-3 1e-3))
                           :name
                           (read-from-string
                            (format nil ":~a-~a-~a" string-name (send joint-class :name) (symbol-name joint-axis)))
                           :weight 0.0
                           ))
             joint-class-list
             joint-axis-list
             )))
     (dotimes (i (1- (length rv-ll)))
       (send (elt rv-ll i) :assoc (elt rv-ll (1+ i))))
     (send self :assoc (car rv-ll))
     (setq links rv-ll)
     ;; 2. setup joint
     (setq rv-jl
           (mapcar
            #'(lambda (joint-class
                       joint-axis
                       parent-link
                       child-link
                       )
                (instance joint-class :init
                          :name
                          (read-from-string
                           (format nil ":~a-~a-~a" string-name (send joint-class :name) (symbol-name joint-axis)))
                          :parent-link parent-link :child-link child-link
                          :axis joint-axis :min -1e10 :max 1e10
                          :max-joint-torque 0.0
                          ))
            joint-class-list
            joint-axis-list
            (butlast rv-ll)
            (cdr rv-ll)
            ))
     (setq joint-list rv-jl)
     (send (elt joint-list 2) :max-angle 0)
     (send-all (subseq joint-list 4) :max-angle angle-of-view)
     (send-all (subseq joint-list 4) :min-angle (- angle-of-view))
     (send self :init-ending)
     ;; 3. set contact-coords and other settings
     (setq _contact-coords
           (make-cascoords
            :parent (car (last rv-ll))
            :coords (orient-coords-to-axis (make-coords) (float-vector 0 0 1) camera-axis)
            ))
     (send self :newcoords (make-coords :pos target-pos))
     self
     ))
  (:contact-coords
   (&rest args)
   "
return $T_{\mathit{cnt}} := \{ \bm{p}_{\mathit{cnt}}, \bm{R}_{\mathit{cnt}} \}$
   "
   (forward-message-to _contact-coords args)
   )
  )


;;;;;;;;;;;;;;;;;;;;;;;;;
;; util
;;;;;;;;;;;;;;;;;;;;;;;;;

(defun attach-additional-end-link
    (&key
     (robot *robot*)
     (target-root-link)
     (target-end-link)
     (translation-axis)
     (rotation-axis)
     (joint-coords (send target-end-link :worldcoords))
     (name (send target-end-link :name))
     )
  "
Attach additional end link to cascaded-link.
This function can be used for attaching the object link to the robot hand link with rotational freedom.
   "
  (let* (additional-ll
         additional-jl
         (string-name
          (string-left-trim ":" (format nil "~a" name)))
         (translation-joint-axis
          (if (atom translation-axis) (ik-arg-axis->axis-list translation-axis) translation-axis))
         (rotation-joint-axis
          (if (atom rotation-axis) (ik-arg-axis->axis-list rotation-axis) rotation-axis))
         (joint-class-list
          (append (make-list (length translation-joint-axis) :initial-element linear-joint)
                  (make-list (length rotation-joint-axis) :initial-element rotational-joint)))
         (joint-axis-list
          (append translation-joint-axis rotation-joint-axis))
         )
    ;; 1. setup link
    (setq additional-ll
          (append
           (list target-root-link)
           (mapcar
            #'(lambda (joint-class
                       joint-axis
                       )
                (instance bodyset-link :init
                          (make-cascoords :coords (send joint-coords :copy-worldcoords))
                          :bodies (list (make-cube 1e-3 1e-3 1e-3))
                          :name
                          (read-from-string
                           (format nil ":~a-~a-~a" string-name (send joint-class :name) (symbol-name joint-axis)))
                          :weight 0.0
                          ))
            ;; butlast is necessary because end-link is given as argument
            (butlast joint-class-list)
            (butlast joint-axis-list)
            )
           (list target-end-link)
           ))
    (dotimes (i (1- (length additional-ll)))
      (send (elt additional-ll i) :assoc (elt additional-ll (1+ i))))
    ;; 2. setup joint
    (setq additional-jl
          (mapcar
           #'(lambda (joint-class
                      joint-axis
                      parent-link
                      child-link
                      )
               (instance joint-class :init
                         :name
                         (read-from-string
                          (format nil ":~a-~a-~a" string-name (send joint-class :name) (symbol-name joint-axis)))
                         :parent-link parent-link :child-link child-link
                         :axis joint-axis :min -1e10 :max 1e10
                         :max-joint-torque 0.0
                         ))
           joint-class-list
           joint-axis-list
           (butlast additional-ll)
           (cdr additional-ll)
           ))
    ;; 3. init-ending
    (mapcar #'(lambda (jnt
                       parent-link
                       child-link
                       )
                (send child-link :add-joint jnt)
                (send child-link :add-parent-link parent-link)
                (send parent-link :add-child-links child-link)
                )
            additional-jl
            (butlast additional-ll)
            (cdr additional-ll)
            )
    ;; set cascaded-link instance
    (send robot :set-val 'links
          (append (send robot :get-val 'links) (cdr additional-ll))) ;; first additional-ll is already contained
    (send robot :set-val 'joint-list
          (append (send robot :get-val 'joint-list) additional-jl))
    (send robot :set-val 'bodies
          (append (send robot :get-val 'bodies) (send target-end-link :bodies)))
    (send robot :update-descendants)
    robot
    ))