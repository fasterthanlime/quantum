
// ours
import quantum/[world, list, collisions]

// third
import dye/[math]
import deadlogger/[Log, Logger]

// sdk
import structs/[ArrayList]

/**
 * A basic quadtree implementation, inspired by:
 * http://gamedev.tutsplus.com/tutorials/implementation/quick-tip-use-quadtrees-to-detect-likely-collisions-in-2d-space/
 */
Quadtree: class {

    logger := static Log getLogger(This name)

    debug := static false
    MAX_OBJECTS := static 8
    MAX_LEVELS := static 8

    level: Int
    objects := ShapeList new()
    bounds: AABB2
    nodes := Quadtree[4] new()

    tempAABB := AABB2 new(0, 0, 0, 0)

    init: func (=level, =bounds)

    /**
     * Clear the quadtree
     */
    clear: func {
        objects clear()

        for (i in 0..nodes length) {
            if (nodes[i] != null) {
                nodes[i] clear()
                nodes[i] = null
            }
        }
    }

    /**
     * Split the Quadtree into 4 sub-nodes
     */
    split: func {
        subWidth  := bounds width  * 0.5
        subHeight := bounds height * 0.5

        xMin := bounds xMin
        yMin := bounds yMin
        xMax := bounds xMax
        yMax := bounds yMax

        // top right
        nodes[0] = This new(level + 1, AABB2 new(
            xMin + subWidth, yMin, xMax, yMin + subHeight))
        // top left
        nodes[1] = This new(level + 1, AABB2 new(
            xMin, yMin, xMin + subWidth, yMin + subHeight))
        // bottom left
        nodes[2] = This new(level + 1, AABB2 new(
            xMin, yMin + subHeight, xMin + subWidth, yMax))
        // bottom right
        nodes[3] = This new(level + 1, AABB2 new(
            xMin + subWidth, yMin + subHeight, xMax, yMax))

        if (debug) {
            logger debug("===============")
            logger debug("Level #{level} split, #{bounds width}, #{bounds height}, bounds = #{bounds}")

            for (i in 0..4) {
                logger debug("Child #{i} bounds: #{nodes[i] bounds}")
            }
        }
    }

    /**
     * Determine which node the object belongs to. -1 means
     * object cannot completely fit within a child node and
     * is part of the parent node.
     */
    getIndex: func (shape: Shape) -> Int {
        index := -1
        shape getAbsoluteAABB(tempAABB)
        verticalMidPoint   := bounds xMin + ((bounds xMax - bounds xMin) * 0.5)
        horizontalMidPoint := bounds yMin + ((bounds yMax - bounds yMin) * 0.5)

        // Object can fit completely within the top quadrants
        topQuadrant := (tempAABB yMin >= bounds yMin && tempAABB yMax <= horizontalMidPoint)

        // Object can fit completely within the bottom quadrants
        bottomQuadrant := (tempAABB yMin >= horizontalMidPoint && tempAABB yMax <= bounds yMax)

        if (tempAABB xMin > bounds xMin && tempAABB xMax < verticalMidPoint) {
            // Object can completely fit within the left quadrants
            if (topQuadrant) {
                index = 1
            } else if (bottomQuadrant) {
                index = 2
            }
        } else if (tempAABB xMin >= verticalMidPoint && tempAABB xMax < bounds xMax) {
            // Object can completely fit within the right quadrants
            if (topQuadrant) {
                index = 0
            } else if (bottomQuadrant) {
                index = 3
            }
        }

        index
    }

    /**
     * Insert the object into the quadtree
     */
    insert: func (shape: Shape) {
        if (shape _quad) {
            raise("adding %p twice in %s, previous was %s" format(shape, _, shape _quad _))
        }

        if (nodes[0] != null) {
            // already split? go ahead and add it to a child
            index := getIndex(shape)

            if (index != -1) {
                nodes[index] insert(shape)
                return
            }
        }

        if (objects contains?(shape)) {
            raise("Quad already contains %p" format(shape))
        }
        objects add(shape)
        shape _quad = this

        if (debug) {
            logger info("Added object, now got #{objects size} objects, max = #{MAX_OBJECTS}, level = #{level} / #{MAX_OBJECTS}, nodes[0] null? #{nodes[0] == null}")
        }

        if (objects size > MAX_OBJECTS && level < MAX_LEVELS) {
            if (nodes[0] == null) {
                split()
            }

            i := 0
            while (i < objects size) {
                index := getIndex(objects fastget(i))
                if (index != -1) {
                    // remove it from us, insert it in child
                    object := objects removeAt(i)
                    object _quad = null
                    nodes[index] insert(object)
                } else {
                    // skip it
                    i += 1
                }
            }
        }
    }

    /**
     * Remove a shape from the quadtree
     */
    remove: func (shape: Shape) {

        if (shape _quad) {
            // TODO: compact?
            success := shape _quad objects remove(shape)

            if (!success) {
                raise("Couldn't remove shape from level-#{shape _quad level} node")
            }

            shape _quad = null
        } else {
            raise("Removing a shape that doesn't have a _quad")
        }
    }

    refresh: func (shape: Shape) {
        index := shape _quad getIndex(shape)
        if (index == -1) {
            // doesn't fit anymore in the quad, refresh!
            remove(shape)
            insert(shape)
        }
    }

    /**
     * Calls f on each shape `shape` might collide with.
     */
    eachNeighbor: func (shape: Shape, f: Func (Shape)) {
        shape getAbsoluteAABB(tempAABB)

        if (nodes[0] != null) {
            // look into our children
            for (i in 0..4) {
                node := nodes[i]
                if (node bounds overlaps?(tempAABB)) {
                    node eachNeighbor(shape, f)
                }
            }
        }

        numShapes := objects size
        for (i in 0..numShapes) {
            object := objects fastget(i)
            if (shape == object) continue // don't test against itself
            f(object)
        }
    }

    printStats: func {
        logger debug("Total objects: #{getTotalObjects()}, Max depth: #{getMaxDepth()}, max children = #{getMaxChildren()}")
    }

    getTotalObjects: func -> Int {
        total := objects size

        if (nodes[0] != null) {
            for (i in 0..4) {
                total += nodes[i] getTotalObjects()
            }
        }

        total
    }

    getMaxDepth: func -> Int {
        if (nodes[0] == null) {
            return level
        }

        maxDepth := 0
        for (i in 0..4) {
            node := nodes[i]
            depth := node getMaxDepth()
            if (depth > maxDepth) {
                maxDepth = depth
            }
        }
        maxDepth
    }

    getMaxChildren: func -> Int {
        if (nodes[0] == null) {
            return objects size
        }

        maxChildren := objects size
        for (i in 0..4) {
            node := nodes[i]
            children := node getMaxChildren()
            if (children > maxChildren) {
                maxChildren = children
            }
        }
        maxChildren
    }

    toString: func -> String {
        "%d / %s" format(level, bounds _)
    }

    _: String { get { toString() } }

    debugFind: func (o: Shape) {
        if (objects indexOf(o) != -1) {
            "Found %p in %s" printfln(o, this _)
        }

        for (i in 0..4) {
            n := nodes[i]
            if (n) {
                n debugFind(o)
            }
        }
    }

}

extend AABB2 {

    overlaps?: func (other: AABB2) -> Bool {
        if (xMax < other xMin) return false
        if (xMin > other xMax) return false
        if (yMax < other yMin) return false
        if (yMin > other yMax) return false
        true
    }

}

