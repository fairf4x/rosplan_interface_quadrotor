(define (domain rosplan_quad)

    (:requirements
        :strips
        :typing
    )

    (:predicates
        (ready)
        (grounded)
        (airborne)
        (finished)
        (squaredone)
    )

    (:action takeoff
        :parameters ()
        :precondition (and (grounded))
        :effect (and (not (grounded)) (ready) (airborne))
    )
    
    (:action land
        :parameters ()
        :precondition (and (airborne) (ready))
        :effect (and    (not (airborne))
                        (grounded)
                        (not (ready))
                        (finished)
                )
    )
    
    (:action flysquare
        :parameters ()
        :precondition (and (airborne))
        :effect (and (squaredone))
    )
)
