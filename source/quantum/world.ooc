
// third-party
use dye
import dye/[math]

use deadlogger
import deadlogger/[Log, Logger]

// sdk
import structs/[ArrayList, HashMap]
import os/Time

// ours
import quantum/[list, quadtree, collisions]

World: class {

    logger := static Log getLogger(This name)

    bodies := ArrayList<Body> new()
    shapes := ShapeList new()
    collisions := Collisions new()
    quad := Quadtree new(0, AABB2 new(0, 0, 1, 1))
    debug := false
    info := CollisionInfo new()

    // adjustable
    gravity := -0.4f
    maxFallVel := -5.0f

    collisionMap := HashMap<Int64, Bool> new()

    locked := false

    init: func {
        // trying optimized bounds
        side := 16384.0f
        pad := 256.0f
        goodBounds := AABB2 new(0.0f, 0.0f, side, side)
        goodBounds add!(vec2(-pad, -pad))

        quad = Quadtree new(0, goodBounds)
    }

    collide: func (step: Float) {
        locked = true

        numTests := -1

        for (b in bodies) {
            b touchesGround = false
        }

        ms := Time measure(||
            numTests = handleQuadCollisions()
        )

        for (b in bodies) {
            b applyAll()
        }

        if (debug) {
            logger info("(quad) #{shapes size} shapes = #{numTests} tests in #{ms}ms")
            quad printStats()
        }

        for (b in bodies) {
            b handleGravityAndFriction(step)
        }

        locked = false
    }

    step: func (delta: Float) {
        locked = true

        for (b in bodies) {
            b step(delta)
        }

        locked = false
    }

    addShape: func (shape: Shape) {
        shapes add(shape)
    }

    addBody: func (b: Body) {
        b world = this
        bodies add(b)
        addShape(b shape)
    }

    removeShape: func (shape: Shape) {
        if (locked) {
            raise("Trying to remove a shape while locked")
        }

        if (shape _quad) {
            quad remove(shape)
            shape _quad = null
        }

        if (!shapes remove(shape)) {
            raise("Couldn't remove shape %p" format(shape))
        }
    }

    removeBody: func (body: Body) {
        if (locked) {
            raise("Trying to remove a body while locked")
        }

        removeShape(body shape)

        if (!bodies remove(body)) {
            raise("Couldn't remove body")
        }
    }

    handleQuadCollisions: func -> Int {
        numTests := 0

        i := 0
        numShapes := shapes size

        // first, update the QuadTree
        Quadtree debug = debug
        for (i in 0..numShapes) {
            shape := shapes fastget(i)
            if (shape _quad) {
                if (shape inert) {
                    // already in and inert. All good!
                } else {
                    // already in but might've moved.
                    quad refresh(shape)
                }
            } else {
                // not in, insert now
                quad insert(shape)
            }
        }

        neighborTest := 0

        // then, run the actual collisions
        for (i in 0..numShapes) {
            a := shapes fastget(i)
            if (a inert) continue // check the other way around

            neighborTest += 1

            // then neighbor collision
            quad eachNeighbor(a, |b|
                aTests := a testsAgainst?(b)
                bTests := b testsAgainst?(a)
                if (!(aTests || bTests)) {
                    return
                }

                aVal := a id as Int64
                bVal := b id as Int64
                if (aVal > bVal) {
                    (aVal, bVal) = (bVal, aVal)
                }

                hash := (aVal << 32 as Int64) | (bVal)
                if (collisionMap contains?(hash)) {
                    // already done
                    return
                }

                numTests += 1

                collisions collide(a, b, info)
                if (info collides) {
                    collisionMap put(hash, true)

                    aReacts := info a reacts?(info, info b)
                    bReacts := info b reacts?(info, info a)

                    if (info a sensor || info b sensor) {
                        // no collision to be applied there
                        return
                    }

                    if (aReacts && a body != null && info normal y > 0.0) {
                        a body touchesGround = true
                    }

                    if (bReacts && b body != null && info normal y < 0.0) {
                        b body touchesGround = true
                    }

                    if (info depth != 0.0) {
                        if (aReacts && bReacts) {
                            weightA := 0.5f
                            weightB := 0.5f
                            if (a body && b body) {
                                weightTotal := a body weight + b body weight
                                weightB = a body weight / weightTotal
                                weightA = b body weight / weightTotal
                            }

                            a react!(info, weightA)
                            info negate!()
                            b react!(info, weightB)
                        } else if (aReacts) {
                            a react!(info, 1.0f)
                        } else if (bReacts) {
                            info negate!()
                            b react!(info, 1.0f)
                        }
                    }
                }
            )
        }

        // clear the map
        collisionMap clear()

        if (debug) {
            logger info("neighborTests = #{neighborTest}")
        }

        numTests
    }

    destroy: func {
        while (!shapes empty?()) {
            s := shapes removeAt(0)
            s destroy()
        }

        while (!bodies empty?()) {
            b := bodies removeAt(0)
            b destroy()
        }
    }

}

CollisionReaction: class {
    diff: Vec2
    other: Shape
    essential := false
    duplicate := false

    init: func (=diff, =other) {
        // muffin
    }
}

Body: class {

    world: World
    shape: Shape
    variableStep := true

    pos := vec2(0, 0)
    hasGravity := true
    weight := 1.0
    friction := 1.0
    elasticity := 0.0

    touchesGround := false

    vel := vec2(0, 0)

    reactions := ArrayList<CollisionReaction> new()

    init: func (=shape) {
        if (!shape) {
            raise("Can't create shapeless body")
        }
        shape body = this
    }

    handleGravityAndFriction: func (delta: Float) {
        if (!variableStep) delta = 1.0f
        if (hasGravity) {
            if (vel y > world maxFallVel) {
                vel y += world gravity * delta
            } else {
                vel y = world maxFallVel
            }
        }
    }

    step: func (delta: Float) {
        if (!variableStep) delta = 1.0f
        pos add!(vel x * delta, vel y * delta)
    }

    react!: func (info: CollisionInfo, weight: Float) {
        if (weight < 0.0f) {
            raise("negative weight!")
        }

        diff := info normal mul(info depth * weight)
        reactions add(CollisionReaction new(diff, info b))
    }

    _v: static func (v: Vec2) -> String {
        "(%.4f, %.4f)" format(v x, v y)
    }

    applyAll: func () {
        if (reactions empty?()) return

        originalPos := vec2(pos)
        info := CollisionInfo new()

        velnp := vel perp() normalized()

        // try to reduce the number of useful reactions
        if (reactions size > 1) for ((i, r1) in reactions) {

            if (r1 duplicate) {
                continue // ignore that twat
            }

            // apply diff + epsilon, see if we've gotten away!
            pos add!(r1 diff mul(1.0f + COLLISION_EPSILON))

            for ((j, r2) in reactions) {
                if (r1 == r2 || r2 duplicate || r2 essential) {
                    continue // ignore that twat
                }

                // test if it solved!
                world collisions collide(shape, r2 other, info)

                if (info collides && info depth > COLLISION_EPSILON) {
                    // still collides with one shape, r2 one
                    // is not a duplicate so far
                } else {
                    r1 essential = true
                    r2 duplicate = true
                }
            }

            pos set!(originalPos)
        }

        // remove duplicates
        {
            iter := reactions iterator()
            while (iter hasNext?()) {
                r := iter next()
                if (r duplicate) {
                    iter remove()
                }
            }
        }

        // add'em up!

        reac := vec2(0, 0)
        for (r in reactions) {
            reac add!(r diff mul(1.0f - COLLISION_EPSILON))
        }
        pos add!(reac)

        // now deal with velocity
        n := reac normalized()
        np := n perp()
        dc := n dot(vel)
        dp := np dot(vel)
        if (dc < 0.0) {
            // off is the component of the velocity *against* the
            // reaction normal - we simply remove it from the velocity
            vel sub!(n mul(dc * (1.0 + elasticity)))

            if (dc < 0.2) {
                vel sub!(np mul(dp * (1.0 - friction)))
            }
        }

        reactions clear()
    }

    destroy: func {
        // not much
        shape = null
    }

}

COLLISION_EPSILON := 0.002f

ShapeGroup: enum from Int {
    NONE
}

Shape: abstract class {

    idSeed := static 0
    id: Int32

    body: Body
    pos := vec2(0, 0)
    trans := vec2(0, 0)
    userData: Object

    listener: CollisionListener
    inert := false
    sensor := false
    group := ShapeGroup NONE

    // reference to the quadtree we are in
    _quad: Quadtree

    init: func {
        id = idSeed
        idSeed += 1
    }

    getAABB: abstract func -> AABB2

    getAbsoluteAABB: func (aabb: AABB2) {
        aabb set!(getAABB())
        aabb add!(pos)
        if (body) {
            aabb add!(body pos)
        }
    }

    getTransform: func -> Vec2 {
        trans set!(pos)
        if (body) {
            trans add!(body pos)
        }
        trans
    }

    testsAgainst?: func (other: Shape) -> Bool {
        if (inert) {
            return false
        }

        if (group != ShapeGroup NONE && other group == group) {
            // same group = no collisions
            return false
        }

        true
    }

    reacts?: func (info: CollisionInfo, other: Shape) -> Bool {
        if (listener) {
            return listener call(info, other)
        }

        if (inert) {
            return false
        }

        true
    }

    react!: func (info: CollisionInfo, weight: Float) {
        if (body) {
            body react!(info, weight)
        }
    }

    subscribe: func (f: Func (CollisionInfo, Shape) -> Bool) {
        listener = CollisionListener new(f)
    }

    destroy: func {
        // not much
        body = null
        listener = null
    }

    cacheData: func {
        // muffin
    }

    getSupportPoint: func (n: Vec2) -> Vec2 {
        raise("Unimplemented: #{class name} getSupportPoint()")
    }

    getSupportEdge: func (n: Vec2) -> Edge {
        raise("Unimplemented: #{class name} getSupportEdge()")
    }

    toString: func -> String {
        "%s: %p" format(class name, this)
    }

}

CollisionListener: class {

    f: Func (CollisionInfo, Shape) -> Bool

    init: func (=f)

    call: func (info: CollisionInfo, shape: Shape) -> Bool {
        f(info, shape)
    }

}

CircleShape: class extends Shape {

    _aabb := AABB2 new(0, 0, 0, 0)

    radius: Float {
        set(=radius) {
            _recomputeAABB()
        }
        get // regular stuff
    }

    init: func (=radius) {
        super()
    }

    getAABB: func -> AABB2 {
        _aabb
    }

    // private stuff

    _recomputeAABB: func {
        _aabb set!(
            -radius, radius,
            -radius, radius
        )
    }

}

AABBShape: class extends Shape {

    aabb: AABB2
    planes: SplittingPlane*
    rawPlanes: SplittingPlane*
    count := 4

    init: func (width, height: Float) {
        super()
        aabb = AABB2 new(width, height)
    }

    // by-ref, kinda evil
    init: func ~aabb (=aabb) {
        super()
    }

    getAABB: func -> AABB2 {
        aabb
    }

    cacheData: func {
        if (!rawPlanes) {
            rawPlanes = gc_malloc(SplittingPlane size * count)
            for (i in 0..count) {
                n := match i {
                    case 0 =>
                        // bottom left to top left (left edge)
                        vec2(-1.0f, 0.0f)
                        case 1 =>
                        // bottom right to bottom left (bottom edge)
                        vec2(0.0f, -1.0f)
                        case 2 =>
                        // top right to bottom right (right edge)
                        vec2(1.0f, 0.0f)
                        case 3 =>
                        // top left to top right (top edge)
                        vec2(0.0f, 1.0f)
                        case =>
                        raise("Normal #{i}? I don't know that.")
                        null
                    }
                    p := (vec2(0, 0), n) as SplittingPlane
                    rawPlanes[i] = p
                }
            }

            if (!planes) {
                // alloc array & content
                planes = gc_malloc(SplittingPlane size * count)
                for (i in 0..count) {
                    p := (vec2(0, 0), vec2(0, 0)) as SplittingPlane
                    planes[i] = p
                }
            }


            // top left
            rawPlanes[0] v0 set!(aabb xMin, aabb yMax)
            // bottom left
            rawPlanes[1] v0 set!(aabb xMin, aabb yMin)
            // bottom right
            rawPlanes[2] v0 set!(aabb xMax, aabb yMin)
            // top right
            rawPlanes[3] v0 set!(aabb xMax, aabb yMax)

            planesValidate(rawPlanes, count)

            trans := getTransform()
            transformPlanes(trans, rawPlanes, count, planes)
        }

        getSupportPoint: func (n: Vec2) -> Vec2 {
            polySupportPoint(planes, count, n)
        }

        getSupportPointIndex: func (n: Vec2) -> Int {
            polySupportPointIndex(planes, count, n)
        }

        getSupportEdge: func (n: Vec2) -> Edge {
            polySupportEdge(planes, count, n)
        }

    }

    // Support edges are the edges of a polygon or segment shape that are in contact.

    Edge: cover {
        a, b: Vec2
        n: Vec2
    }

    PolyShape: class extends Shape {

        aabb: AABB2
        count: Int
        rawPlanes: SplittingPlane*
        planes: SplittingPlane*

        init: func (data: Vec2*, =count) {
            super()
            aabb = AABB2 new()

            polySanitize!(data, count)

            rawPlanes = gc_malloc(SplittingPlane size * count)
            for (i in 0..count) {
                a := data[i]
                b := data[(i + 1) % count]
                n := a sub(b) rperp() normalized()
                v0 := a clone()
                rawPlanes[i] = (v0, n) as SplittingPlane
                aabb expand!(v0)
            }

            planesValidate(rawPlanes, count)
        }

        cacheData: func {
            if (!planes) {
                // alloc array & content
                planes = gc_malloc(SplittingPlane size * count)
                for (i in 0..count) {
                    p := (vec2(0, 0), vec2(0, 0)) as SplittingPlane
                    planes[i] = p
                }
            }

            trans := getTransform()
            transformPlanes(trans, rawPlanes, count, planes)
        }

        getAABB: func -> AABB2 {
            aabb
        }

        getSupportPoint: func (n: Vec2) -> Vec2 {
            polySupportPoint(planes, count, n)
        }

        getSupportPointIndex: func (n: Vec2) -> Int {
            polySupportPointIndex(planes, count, n)
        }

        getSupportEdge: func (n: Vec2) -> Edge {
            polySupportEdge(planes, count, n)
        }

    }

    transformPlanes: func (trans: Vec2, rawPlanes: SplittingPlane*, count: Int, planes: SplittingPlane*) {
        for (i in 0..count) {
            pl := rawPlanes[i]
            v0 := pl v0
            n  := pl n

            // copy
            planes[i] v0 set!(v0)
            planes[i] n set!(n)

            // transform
            planes[i] v0 add!(trans)
        }
    }

    polySupportPoint: func (planes: SplittingPlane*, count: Int, n: Vec2) -> Vec2 {
        planes[polySupportPointIndex(planes, count, n)] v0
    }

    // Support points are the maximal points on a shape's perimeter along a certain axis.
    // The GJK and EPA algorithms use support points to iteratively sample the surface of the two shapes' minkowski difference.
    polySupportPointIndex: func (planes: SplittingPlane*, count: Int, n: Vec2) -> Int {
        max: Float = 0.0f - INFINITY
        index := 0

        for (i in 0..count) {
            v := planes[i] v0
            d := v dot(n)
            if (d > max) {
                max = d
                index = i
            }
        }

        index
    }

    polySupportEdge: func (planes: SplittingPlane*, count: Int, n: Vec2) -> Edge {
        i1 := polySupportPointIndex(planes, count, n)
        i0 := (i1 - 1 + count) % count
        i2 := (i1 + 1) % count

        i1n := planes[i0] n
        i2n := planes[i1] n
        dot1 := n dot(i1n)
        dot2 := n dot(i2n)

        if (dot1 > dot2) {
            return (planes[i0] v0, planes[i1] v0, i1n) as Edge
        } else {
            return (planes[i1] v0, planes[i2] v0, i2n) as Edge
        }
    }

    planesValidate: func (planes: SplittingPlane*, count: Int) {
        for (i in 0..count) {
            a := planes[i] v0
            b := planes[(i + 1) % count] v0
            c := planes[(i + 2) % count] v0

            if (b sub(a) cross(c sub(a)) < 0.0f) {
                raise("Invalid poly")
            }
        }

        // all good
    }

    polySanitize!: func (vecs: Vec2*, count: Int) {
        valid := true

        for (i in 0..count) {
            a := vecs[i]
            b := vecs[(i + 1) % count]
            c := vecs[(i + 2) % count]

            if (b sub(a) cross(c sub(a)) < 0.0f) {
                valid = false
                break
            }
        }

        if (!valid) {
            // reveeeeeeeeerse o/
            bytes := Pointer size * count
            copy := gc_malloc(bytes) as Vec2*
            memcpy(copy, vecs, bytes)

            for (i in 0..count) {
                j := count - 1 - i
                vecs[i] = copy[j]
            }
        }
    }

    SplittingPlane: cover {
        // v0 = vector, n = normal
        v0, n: Vec2
    }

