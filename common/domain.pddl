(define (domain rosplan_quad)

    (:requirements
        :strips
        :typing
    )

    (:predicates
        (grounded ?q - quad)
        (airborne ?q - quad)
        (finished ?q -quad)
        (squaredone ?q - quad)
    )
    
    (:types
      quad - object
    )

    (:action takeoff
        :parameters (?q - quad)
        :precondition (and (grounded ?q))
        :effect (and (not (grounded ?q)) (airborne ?q))
    )
    
    (:action land
        :parameters (?q - quad)
        :precondition (and (airborne ?q))
        :effect (and    (not (airborne ?q))
                        (finished ?q)
                        (grounded ?q)
                )
    )
    
    (:action flysquare
        :parameters (?q - quad)
        :precondition (and (airborne ?q))
        :effect (and (squaredone ?q))
    )
)
