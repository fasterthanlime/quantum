
// sdk
import structs/[ArrayList]

// ours
import quantum/[world]

/**
 * A special-purpose ShapeList class for performance...
 * until generics are propertly inlined in ooc :)
 */
ShapeList: class extends ArrayList<Shape> {

    init: func {
        super()
    }

    fastget: final inline func (index: Int) -> Shape {
        (data as Shape*)[index]
    }

}

