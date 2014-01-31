
// third-party
use dye
import dye/[core, math, primitives, sprite, text]

use deadlogger
import deadlogger/[Log, Logger]

// ours
import quantum/[world]

/**
 * Contact information
 */
CollisionInfo: class {

    collides := false
    contact := vec2(0, 0)
    normal := vec2(0, 0)
    depth := 0.0
    a, b: Shape

    init: func

    reset!: func {
        collides = false
        contact set!(0, 0)
        normal set!(0, 0)
        depth = 0.0
        a = null
        b = null
    }

    negate!: func {
        normal mul!(-1.0f)
        tmp := a
        a = b
        b = tmp
    }

    toString: func -> String {
        match collides {
            case true =>
                "at #{contact}, normal #{normal}, depth #{depth}"
            case false =>
                "no collision"
        }
    }

}

MAX_GJK_ITERATIONS := static 30
MAX_EPA_ITERATIONS := static 30
WARN_GJK_ITERATIONS := static 20
WARN_EPA_ITERATIONS := static 20

__FILE__: extern CString
__LINE__: extern Int

/**
 * Various collision handlers
 */
Collisions: class {

    debugGroup := static GlGroup new()
    draws := static true

    // debug stuff
    drawPoint: static func (pos: Vec2, color: Color) {
        if (!draws) return
        r := GlRectangle new(vec2(12, 12))
        r pos set!(pos)
        r color set!(color)
        r opacity = 0.7
        debugGroup add(r)
    }

    drawAABB: static func (aabb: AABB2, color: Color) {
        drawBox(aabb center(), aabb size, color)
    }

    drawBox: static func (pos: Vec2, size: Vec2, color: Color) {
        if (!draws) return
        r := GlRectangle new(size)
        r pos set!(pos)
        r color set!(color)
        r opacity = 0.8
        r filled = false
        debugGroup add(r)
    }

    drawArrow: static func (pos: Vec2, dir: Vec2, color: Color) {
        if (!draws) return
        s := GlSprite new("assets/png/arrow.png")
        s pos set!(pos)
        s angle = dir angle() toDegrees()
        s color set!(color)
        sc := 0.6
        s scale set!(sc, sc)
        s opacity = 0.7
        debugGroup add(s)
    }

    drawText: static func (pos: Vec2, text: String, color: Color) {
        if (!draws) return
        t := GlText new("assets/ttf/DroidSansMono.ttf", text, 12)
        t pos set!(pos)
        t color set!(color)
        debugGroup add(t)
    }

    logger := static Log getLogger(This name)

    // write collisions for everything here o/
    boxA := AABB2 new()
    boxB := AABB2 new()

    init: func

    collide: func (a, b: Shape, info: CollisionInfo) {
        match a {
            case aabb1: AABBShape =>
                match b {
                    case aabb2: AABBShape =>
                        // most frequent case
                        polyToPoly(aabb1, aabb2, info)
                    case poly2: PolyShape =>
                        polyToPoly(aabb1, poly2, info)
                }
            case poly1: PolyShape =>
                match b {
                    case aabb2: AABBShape =>
                        polyToPoly(poly1, aabb2, info)
                    case poly2: PolyShape =>
                        polyToPoly(poly1, poly2, info)
                }
        }
    }

    polyToPoly: final func (poly1, poly2: Shape, info: CollisionInfo) {
        info reset!()

        // quickly eliminate with a trivial AABB test
        if (!aabbTest(poly1, poly2)) {
            // no dice
            return
        }

        // do the real thing now
        info a = poly1
        info b = poly2

        poly1 cacheData()
        poly2 cacheData()

        context := (poly1, poly2) as SupportContext
        points := GJK(context)

        if (points d > 0.0) return

        n := points n
        e1 := poly1 getSupportEdge(n neg())
        e2 := poly2 getSupportEdge(n)
        ContactPoints(e1, e2, points, info)

        if (!info collides) return
    }

    aabbTest: final func (a, b: Shape) -> Bool {
        // grab absolute AABBs
        boxA set!(a getAABB())
        boxB set!(b getAABB())
        boxA add!(a getTransform())
        boxB add!(b getTransform())

        // Test 4 separating planes
        if (boxA yMin > boxB yMax ||
            boxB yMin > boxA yMax ||
            boxB xMin > boxA xMax ||
            boxA xMin > boxB xMax) {
            return false
        }

        // AABBs overlap
        true
    }

    GJK: static func (ctx: SupportContext) -> ClosestPoints {
        //debug("===================== CJK!")
        aabb1 := ctx shape1 getAABB() add(ctx shape1 getTransform())
        aabb2 := ctx shape2 getAABB() add(ctx shape2 getTransform())

        // No cached indexes, use the shapes' bounding box centers as a guess for a starting axis.
        axis := aabb1 center() sub(aabb2 center()) perp()
        v0 := Support(ctx, axis)
        v1 := Support(ctx, axis neg())

        GJKRecurse(ctx, v0, v1, 1)
    }

    // Recursive implementation of the GJK loop.
    GJKRecurse: static func (ctx: SupportContext, v0, v1: MinkowskiPoint, iteration: Int) -> ClosestPoints {
        if (iteration > MAX_GJK_ITERATIONS) {
            if (iteration >= WARN_GJK_ITERATIONS) {
                //logger warn("#{__FILE__}:#{__LINE__} High GJK iterations: #{iteration}")

                aabb1 := ctx shape1 getAABB() add(ctx shape1 getTransform())
                aabb2 := ctx shape2 getAABB() add(ctx shape2 getTransform())
                Collisions drawAABB(aabb1, Color new(0, 0, 255))
                Collisions drawAABB(aabb2, Color new(0, 255, 0))
            }
            return ClosestPoints new(v0, v1)
        }

        delta := v1 ab sub(v0 ab)
        // TODO: should this be an area2x check?
        if(delta cross(v0 ab add(v1 ab)) > 0.0f) {
            //debug("origin is behind")
            // Origin is behind axis. Flip and try again
            return GJKRecurse(ctx, v1, v0, iteration)
        } else {
            t := ClosestT(v0 ab, v1 ab)
            n := (-1.0f < t && t < 1.0f ? delta perp() : LerpT(v0 ab, v1 ab, t) neg())
            p := Support(ctx, n)

            if (
                v1 ab sub(p ab) cross(v1 ab add(p ab)) > 0.0f &&
                v0 ab sub(p ab) cross(v0 ab add(p ab)) < 0.0f
            ) {
                // The triangle v0, p, v1 contains the origin. Use EPA to find the MSA.
                if (iteration >= WARN_GJK_ITERATIONS) {
                    logger warn("#{__FILE__}:#{__LINE__} High GJK->EPA iterations: #{iteration}")
                }

                return EPA(ctx, v0, p, v1)
            } else {
                // The new point must be farther along the normal than the existing points.
                pdotn := p ab dot(n)
                maxdot := fmax(v0 ab dot(n), v1 ab dot(n))
                if (pdotn <= maxdot) {
                    // The edge v0, v1 that we already have is the closest to (0, 0) since p was not closer.
                    if (iteration >= WARN_GJK_ITERATIONS) {
                        logger warn("#{__FILE__}:#{__LINE__} High GJK iterations: #{iteration}")
                    }
                    return ClosestPoints new(v0, v1)
                } else {
                    d1 := ClosestDist(v0 ab, p ab)
                    d2 := ClosestDist(p ab, v1 ab)
                    // p was closer to the origin than our existing edge.
                    // Need to figure out which existing point to drop.
                    if (d1 < d2) {
                        return GJKRecurse(ctx, v0, p, iteration + 1)
                    } else {
                        return GJKRecurse(ctx, p, v1, iteration + 1)
                    }
                }
            }
        }
    }

    EPA_DEBUG        := static false
    GJK_DEBUG        := static false
    COLLISIONS_DEBUG := static false

    debug: static func (msg: String) {
        if (COLLISIONS_DEBUG) {
            logger debug(msg)
        }
    }

    EPA: static func (ctx: SupportContext, v0, v1, v2: MinkowskiPoint) -> ClosestPoints {
        hull: MinkowskiPoint* = [v0, v1, v2]
        return EPARecurse(ctx, 3, hull, 1)
    }

    // Recursive implementation of the EPA loop.
    // Each recursion adds a point to the convex hull until it's known that we have the closest point on the surface.
    EPARecurse: static func (ctx: SupportContext, count: Int, hull: MinkowskiPoint*, iteration: Int) -> ClosestPoints {
        mini := 0
        minDist: Float = INFINITY

        // TODO: precalculate this when building the hull and save a step
        // Find the closest segment hull[i] and hull[i + 1] to (0, 0)
        j := 0
        i := count - 1

        while (j < count) {
            d := ClosestDist(hull[i] ab, hull[j] ab)
            if (d < minDist) {
                minDist = d
                mini = i
            }
            i = j
            j += 1
        }

        v0 := hull[mini]
        v1 := hull[(mini + 1) % count]
        if (v0 ab equals?(v1 ab)) {
            logger error("Internal error: EPA vertices are the same (%d and %d)", mini, (mini + 1) % count)
        }
        // Check if there is a point on the minkowski difference beyond this edge.
        p := Support(ctx, v1 ab sub(v0 ab) perp())

        // The signed area of the triangle [v0.ab, v1.ab, p] will be positive if p lies beyond v0.ab, v1.ab.
        area2x := v1 ab sub(v0 ab) cross(p ab sub(v0 ab) add(p ab sub(v1 ab)))

        if (area2x > 0.0f && iteration < MAX_EPA_ITERATIONS) {
            // Rebuild the convex hull by inserting p.
            count2 := 1
            hull2 := gc_malloc((count + 1) * MinkowskiPoint size) as MinkowskiPoint*
            hull2[0] = p

            for (i in 0..count) {
                index := (mini + 1 + i) % count

                h0 := hull2[count2 - 1] ab
                h1 := hull[index] ab
                h2 := (i + 1 < count ? hull[(index + 1) % count] : p) ab

                // TODO: should this be changed to an area2x check?
                if (h2 sub(h0) cross(h1 sub(h0)) > 0.0f) {
                    hull2[count2] = hull[index]
                    count2 += 1
                }
            }

            return EPARecurse(ctx, count2, hull2, iteration + 1)
        } else {
            // Could not find a new point to insert, so we have found the closest edge of the minkowski difference.
            if (iteration >= WARN_EPA_ITERATIONS) {
                logger warn("High EPA iterations: #{iteration}")
            }

            return ClosestPoints new(v0, v1)
        }
    }

    ContactPoints: static func (e1, e2: Edge, points: ClosestPoints, info: CollisionInfo) {
        minDist := 0.0f
        if (points d > minDist) {
            return
        }

        n := points n neg()
        info normal set!(points n)

        // Distances along the axis parallel to n
        d_e1_a := e1 a cross(n)
        d_e1_b := e1 b cross(n)
        d_e2_a := e2 a cross(n)
        d_e2_b := e2 b cross(n)

        e1_denom := 1.0f / (d_e1_b - d_e1_a)
        e2_denom := 1.0f / (d_e2_b - d_e2_a)

        {
            p1 := e1 a lerp(e1 b, fclamp01((d_e2_b - d_e1_a) * e1_denom))
            p2 := e2 a lerp(e2 b, fclamp01((d_e1_a - d_e2_a) * e2_denom))
            dist := p2 sub(p1) dot(n)

            if (dist <= 0.0f) {
                info contact set!(p1)
                info depth = -dist
                info collides = true
            }
        }

        {
            p1 := e1 a lerp(e1 b, fclamp01((d_e2_a - d_e1_a) * e1_denom))
            p2 := e2 a lerp(e2 b, fclamp01((d_e1_b - d_e2_a) * e2_denom))
            dist := p2 sub(p1) dot(n)

            if (dist <= 0.0f) {
                info contact set!(p1)
                info depth = -dist
                info collides = true
            }
        }

        return
    }

}

// Calculate the maximal point on the minkowski difference of two shapes along a particular axis.
Support: static func (ctx: SupportContext, n: Vec2) -> MinkowskiPoint {
    a := ctx shape1 getSupportPoint(n neg())
    b := ctx shape2 getSupportPoint(n)
    MinkowskiPoint new(a, b)
}

ClosestPoints: cover {
    // Surface points in absolute coordinates.
    a, b: Vec2
    // Minimum separating axis of the two shapes.
    n: Vec2
    // Signed distance between the points.
    d: Float

    // Calculate the closest points on two shapes given the closest edge on their minkowski difference to (0, 0)
    new: static func (v0, v1: MinkowskiPoint) -> This {
        // Find the closest p(t) on the minkowski difference to (0, 0)
        t := ClosestT(v0 ab, v1 ab)
        p := LerpT(v0 ab, v1 ab, t)

        // Interpolate the original support points using the same 't' value as above.
        // This gives you the closest surface points in absolute coordinates. NEAT!
        pa := LerpT(v0 a, v1 a, t)
        pb := LerpT(v0 b, v1 b, t)

        // First try calculating the MSA from the minkowski difference edge.
        // This gives us a nice, accurate MSA when the surfaces are close together.
        delta := v1 ab sub(v0 ab)
        n := delta perp() normalized()
        d := -(n dot(p))

        if (d <= 0.0f || (-1.0f < t && t < 1.0f)) {
            // If the shapes are overlapping, or we have a regular vertex/edge collision, we are done.
            points := (pa, pb, n, d) as This
            return points
        } else {
            // Vertex/vertex collisions need special treatment since the MSA won't be shared with an axis of the minkowski difference.
            d2 := p norm()
            n := p mul(1.0f / (d2 + FLT_MIN))
            points := (pa, pb, n, d2) as This
            return points
        }
    }
}

SupportContext: cover {
    shape1, shape2: Shape
}

// A point on the surface of two shape's minkowski difference.
MinkowskiPoint: cover {
    // Cache the two original support points.
    a, b: Vec2

    // b - a
    ab: Vec2

    init: func@ (=a, =b) {
        ab = b sub(a)
    }

    toString: func -> String {
        "(a #{a}, b #{b})"
    }
}

// Find the closest p(t) to (0, 0) where p(t) = a*(1-t)/2 + b*(1+t)/2
// The range for t is [-1, 1] to avoid floating point issues if the parameters are swapped.
ClosestT: func (a, b: Vec2) -> Float {
    delta := b sub(a)
    -(delta dot(a add(b)) / delta squaredNorm()) clamp(-1.0f, 1.0f)
}

// Basically the same as cpvlerp(), except t = [-1, 1]
LerpT: func (a, b: Vec2, t: Float) -> Vec2 {
    ht := 0.5f * t
    a mul(0.5f - ht) add(b mul(0.5f + ht))
}

ClosestDist: func (v0, v1: Vec2) -> Float {
    LerpT(v0, v1, ClosestT(v0, v1)) squaredNorm()
}

fmax: func (a, b: Float) -> Float {
    if (a > b) return a
    b
}

fclamp01: func (x: Float) -> Float {
    if (x < 0.0f) return 0.0f
    if (x > 1.0f) return 1.0f
    x
}

