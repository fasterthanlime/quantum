
use quantum
import quantum/[world]

main: func {

    world := World new()

    box := AABBShape new(5, 5)
    body := Body new(box)
    world addBody(body)

    obstacle := AABBShape new(40, 5)
    obstacle pos set!(0, -40)
    world addShape(obstacle)

    step := 1.0f

    for (i in 0..20) {
        world collide(step)
        world step(step)
        "pos: #{body pos}\t vel = #{body vel}" println()
    }

}
