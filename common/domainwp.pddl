(define (domain rosplan_quad)

    (:requirements :strips :typing :fluents)
        
    (:types
      quad - object
      waypoint - object
    )
    
    (:predicates
        (grounded ?q - quad)
        (airborne ?q - quad)
        (squaredone ?q - quad)
        (at_waypoint ?q - quad ?w - waypoint)
        (visited ?q - quad ?w - waypoint)
	      (connected ?from ?to - waypoint)
    )

    (:functions
	    (distance ?wp1 ?wp2 - waypoint) 
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
                        (grounded ?q)
                )
    )

    (:action fly_to_waypoint
        :parameters (?q - quad ?from ?to - waypoint)
        :precondition (and  (airborne ?q)
                            (at_waypoint ?q ?from)
                      )
        :effect (and  (not (at_waypoint ?q ?from))
                      (at_waypoint ?q ?to)
                      (visited ?q ?to)
                )
    )
    
    (:action flysquare
        :parameters (?q - quad)
        :precondition (and (airborne ?q))
        :effect (and (squaredone ?q))
    )
)
