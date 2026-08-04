"""
Single source of truth for world/simulation data shared across agents,
keyed by world_name:
 
- LOCATIONS      — named navigation waypoints (x, y, yaw) used by
                   mobile agents (Human, Tiago, future ones) via goto().
- DEFAULT_SCENES — hand-placed pick/place spots used by FrankaResearch3's
                   scene-based tasks (grill, chopping boards, etc.).
 
This lives next to utils.py but stays in its own file on purpose:
it's simulation/world data (tied to the .sdf world layout), not a
general-purpose helper — keeping it separate means changing one
table's coordinates never requires touching unrelated code, and a
diff to this file is immediately recognizable as "the world moved."
 
Keyed by world_name (Human, Tiago, and FrankaResearch3 already receive
this as a constructor arg and store it as self._world_name), so callers
go through get_locations()/get_scene() rather than indexing the dicts
directly. Add a new top-level key for each new world. Keep names
consistent across worlds where the same semantic point exists (e.g.
every world should probably have a "House") so agent code doesn't need
to special-case which world it's in.
"""

# Tiago and Human can get it automatically, but they get the center of the objects.
# Here I define some specific places just to look better in the video
LOCATIONS = {
    "backyard": {
        "DiningTable":      {"x": -0.77, "y": -1.54, "yaw": 3.14},
        "House":            {"x": -0.19, "y": -6.76, "yaw": 1.57},
        "MainEntrance":     {"x": -0.19, "y": -6.76, "yaw": 1.57},
        "GroupOfGuests_1":  {"x":  3.25, "y": -2.82, "yaw": 0.0},
        "GroupOfGuests_2":  {"x": -2.21, "y":  3.54, "yaw": 2.15},
        "MainPrepTable":    {"x":  0.95, "y":  4.30, "yaw": 1.57},
        "GrillPrepTable":   {"x":  4.14, "y":  4.24, "yaw": 0.8},
        "DrinksTable":      {"x": -4.50, "y":  4.89, "yaw": -1.55},
    },
    "backyard_b": {
        "DiningTable_1":        {"x":  2.00, "y":  4.45, "yaw":  2.11},
        "DiningTable_2":        {"x":  2.00, "y":  0.96, "yaw":  1.03},
        "House":                {"x":  0.26, "y":  7.75, "yaw":  1.57},
        "MainEntrance":         {"x":  1.56, "y": -6.70, "yaw": -3.14},
        "GroupOfGuests_1":      {"x": -4.63, "y": -1.34, "yaw": -1.32},
        "GroupOfGuests_2":      {"x":  4.70, "y": -1.55, "yaw":  0.93},
        "SaladPrepTable":       {"x": -4.48, "y":  4.95, "yaw": -1.57},
        "VegetablePrepTable":   {"x": -4.48, "y":  1.93, "yaw": -1.57},
        "FoodGrillPrepTable":   {"x": -3.58, "y": -5.33, "yaw": -1.57},
        "FishGrillPrepTable":   {"x":  3.44, "y": -5.33, "yaw":  1.57},
        "DrinksTable":          {"x": -3.03, "y":  5.12, "yaw":  3.0},
    },
}

def get_locations(world_name: str) -> dict:
    """Look up the location table for a given world, with a clear
    error if the world hasn't been defined yet (instead of a raw
    KeyError deep inside goto())."""
    try:
        return LOCATIONS[world_name]
    except KeyError:
        raise KeyError(
            f"No locations defined for world '{world_name}'. "
            f"Known worlds: {list(LOCATIONS.keys())}"
        )

# ---------------------------------------------------------------------------
# FR3 pick/place scenes
# ---------------------------------------------------------------------------
# FR3 can resolve object poses automatically via ObjectToRobot, but these
# hand-placed spots exist purely to look better on camera.
DEFAULT_SCENES = {
    "backyard": {
        "food_grill": {
            "placements": {
                "meat_1":         {"place_grill": (5.0, 4.65, 0.885), "place_plate": (4.30, 5.26, 0.90)},
                "meat_2":         {"place_grill": (5.2, 4.65, 0.885), "place_plate": (4.20, 5.18, 0.90)},
                "meat_3":         {"place_grill": (4.8, 4.65, 0.885), "place_plate": (4.38, 5.15, 0.90)},
                "garlic_bread_1": {"place_grill": (5.1, 4.47, 0.900), "place_plate": (4.25, 5.21, 0.92)},
                "garlic_bread_2": {"place_grill": (4.9, 4.47, 0.900), "place_plate": (4.34, 5.21, 0.92)},
            }
        },
        "vegetables_side": {
            "placements": {
                "carrot_1":         {"place_chop": (1.12, 4.90, 0.91), "place_bowl": (1.47, 4.90, 0.89)},
                #"carrot_2":        {"place_chop": (1.04, 4.90, 0.91), "place_bowl": (1.54, 4.90, 0.89)},
                #"cucumber_1":       {"place_chop": (0.94, 4.90, 0.91), "place_bowl": (1.50, 4.90, 0.89)},
                "cucumber_2":       {"place_chop": (0.86, 4.90, 0.91), "place_bowl": (1.59, 4.90, 0.89)},
            }
        },
        "salads_side": {
            "placements": {
                "tomato_1":         {"place_chop": (0.9, 4.85, 0.91), "place_bowl": (0.52, 4.84, 0.89)},
                #"tomato_2":         {"place_chop": (1.11, 4.84, 0.91), "place_bowl": (0.53, 4.96, 0.89)},
                #"tomato_3":         {"place_chop": (1.07, 4.97, 0.91), "place_bowl": (0.64, 4.84, 0.89)},
                "purple_onion_1":   {"place_chop": (0.95, 4.95, 0.91), "place_bowl": (0.64, 4.96, 0.89)},
            }
        },
        "drinks": {
            "placements": {
                "drink_1":  {"place_guests": (-2.68, -0.72, 0.30)}, # Dinning table, guest 1
                #"drink_2":  {"place_guests": (-1.11, -0.72, 0.30)}, # Dinning table, guest 2
                #"drink_3":  {"place_guests": (-2.92, -0.72, 0.30)}, # Dinning table, guest 3
                #"drink_4":  {"place_guests": (-1.34, -0.60, 0.30)}, # Dinning table, guest 4

                "drink_5":  {"place_guests": (4.74, -3.31, 0.30)}, # GroupOfGuests1, guests 5
                #"drink_6":  {"place_guests": (3.8, -2.36, 0.30)}, # GroupOfGuests1, guests 6
                #"drink_7":  {"place_guests": (3.54, -3.62, 0.30)}, # GroupOfGuests1, guests 7

                "drink_7":  {"place_guests": (-1.90, 4.66, 0.30)}, # GroupOfGuests2, troquei pra ir em todos os grupos -- dps remover essa linha

                # Removed from Gazebo to improve performance
                #"drink_8":  {"place_guests": (-1.90, 4.66, 0.30)}, # GroupOfGuests2, guests 8
                #"drink_9":  {"place_guests": (-1.30, 4.11, 0.30)}, # GroupOfGuests2, guests 9
                #"drink_10": {"place_guests": (-3.14, -0.72, 0.30)}, # Dinning Table, guests 10
            }
        }
    },
    "backyard_b": {
        "food_grill": {
            "placements": {
                "meat_1":           {"place_grill": (-4.65, -4.63, 0.885), "place_plate": (-4.11, -5.07, 0.90)},
                "meat_2":           {"place_grill": (-4.77, -4.63, 0.885), "place_plate": (-4.18, -5.07, 0.90)},
                "meat_3":           {"place_grill": (-4.90, -4.63, 0.885), "place_plate": (-4.16, -4.94, 0.90)},
                "garlic_bread_1":   {"place_grill": (-4.71, -4.48, 0.900), "place_plate": (-4.28, -5.00, 0.92)},
                "garlic_bread_2":   {"place_grill": (-4.84, -5.44, 0.900), "place_plate": (-4.33, -5.00, 0.92)},
            }
        },
        "seafood_grill": {
            "placements": {
                "fish_1":           {"place_grill": (4.80, -4.65, 0.89), "place_plate": (3.90, -5.04, 0.91)},
                "fish_2":           {"place_grill": (4.60, -4.65, 0.89), "place_plate": (3.94, -5.09, 0.91)},
                "fish_3":           {"place_grill": (4.41, -4.65, 0.89), "place_plate": (3.99, -5.03, 0.91)},
                "squid_1":          {"place_grill": (4.74, -4.50, 0.88), "place_plate": (4.03, -5.11, 0.90)},
                "squid_2":          {"place_grill": (4.61, -4.50, 0.88), "place_plate": (4.09, -5.01, 0.90)},
                "squid_3":          {"place_grill": (4.49, -4.50, 0.88), "place_plate": (4.16, -5.07, 0.90)},
            }
        },
        "vegetables_side": {
            "placements": {
                "carrot_1":         {"place_chop": (-5.02, 2.21, 0.91), "place_bowl": (-5.03, 1.46, 0.89)},
                "cucumber_2":       {"place_chop": (-5.02, 2.12, 0.91), "place_bowl": (-5.03, 1.52, 0.89)},
                "carrot_2":         {"place_chop": (-5.02, 2.04, 0.91), "place_bowl": (-5.03, 1.59, 0.89)},
                "cucumber_1":       {"place_chop": (-5.03, 1.96, 0.91), "place_bowl": (-5.03, 1.66, 0.89)},
            }
        },
        "salads_side": {
            "placements": {
                "tomato_1":         {"place_chop": (-4.99, 5.14, 0.91), "place_bowl": (-4.99, 4.66, 0.89)},
                "tomato_2":         {"place_chop": (-5.00, 5.30, 0.91), "place_bowl": (-5.00, 4.80, 0.89)},
                "tomato_3":         {"place_chop": (-5.10, 5.21, 0.91), "place_bowl": (-5.10, 4.67, 0.89)},
                "purple_onion_1":   {"place_chop": (-5.11, 5.38, 0.91), "place_bowl": (-5.11, 4.80, 0.89)},
            }
        },
        "drinks": {
            "placements": {
                "drink_1":  {"place_guests": (2.25, 0.23, 0.30)}, # Dinning table, woman sit 1
                "drink_5":  {"place_guests": (2.03, 2.74, 0.30)}, # GroupOfGuests3, guests 6
                "drink_7":  {"place_guests": (-4.71, -1.50, 0.30)}, # GroupOfGuests1, troquei pra ir em todos os grupos -- dps remover essa linha
            }
        }
    },
}
 
 
def get_scene(world_name) -> dict:
    """Look up the FR3 pick/place scene layout for a world. Falls back
    to the 'backyard' layout if world_name is unset or not yet defined
    — this mirrors FrankaResearch3's original fallback behavior, since
    a missing scene is cosmetic (camera framing) rather than fatal like
    a missing navigation waypoint."""
    return DEFAULT_SCENES.get(world_name, DEFAULT_SCENES["backyard"])