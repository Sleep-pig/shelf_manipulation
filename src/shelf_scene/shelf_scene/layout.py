import random


def layout(seed, width=.36, depth=.32):
    """Clear inner width and outer shelf depth, in metres; front edge stays fixed."""
    if not .24 <= width <= .50 or not .23 <= depth <= .40:
        raise ValueError("Supported shelf widths: 0.24–0.50 m; depths: 0.23–0.40 m")
    center_x = .37 + depth/2
    side_y = width/2 + .01
    rng = random.Random(seed)
    return [
        ("floor", (depth, width+.04, .02), (center_x, 0., .20), 0.),
        ("ceiling", (depth, width+.04, .02), (center_x, 0., .48), 0.),
        ("left", (depth, .02, .26), (center_x, side_y, .34), 0.),
        ("right", (depth, .02, .26), (center_x, -side_y, .34), 0.),
        ("back", (.02, width, .26), (.37+depth-.01, 0., .34), 0.),
        ("target", (.07, .04, .06),
         (rng.uniform(.44, .37+depth-.10), rng.uniform(max(-.07, -.5*width+.045), min(.03, .5*width-.12)), .24),
         rng.uniform(-.6, .6)),
        ("obstacle", (.08, .05, .12), (.49, width/2-.05, .27), 0.),
        ("tray", (.20, .18, .02), (.32, -.34, .15), 0.),
    ]
