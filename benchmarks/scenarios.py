"""Reachable guardian starts in the same maze.world (world and map frames)."""

SCENARIOS = {
    1: (2.5, 2.5),
    2: (1.5, 1.5),
    3: (-2.5, 1.5),
}

FIRST_SPAWN = (-0.34, 0.4)
ARENA_WORLD_BOUNDS = (-3.0, 0.0, 3.0, 5.0)


def scenario_environment(number):
    x, y = SCENARIOS[number]
    # Both start polygons and arena bounds are expressed in the shifted map frame.
    cx, cy = x - FIRST_SPAWN[0], y - FIRST_SPAWN[1]
    corners = ((cx - .25, cy - .25), (cx + .25, cy - .25),
               (cx + .25, cy + .25), (cx - .25, cy + .25))
    polygon = '[' + ','.join(f'{value:.2f}' for corner in corners
                           for value in corner) + ']'
    arena = '[' + ','.join(str(round(value - FIRST_SPAWN[index % 2], 2))
                           for index, value in enumerate(ARENA_WORLD_BOUNDS)) + ']'
    return {"SPAWN_X": str(FIRST_SPAWN[0]),
            "DUEL_SPAWN_Y": str(FIRST_SPAWN[1]),
            "OPPONENT_X": str(x), "OPPONENT_Y": str(y),
            "OPPONENT_YAW": "0" if x < 0 else "3.14159",
            "DUEL_SECOND_START": polygon,
            "DUEL_ARENA_BOUNDS": arena, "DUEL_SCENARIO": str(number)}
