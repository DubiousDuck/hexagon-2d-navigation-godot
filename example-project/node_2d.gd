extends Node2D

@export var tile_map: TileMapLayer

@onready var char = $CharacterBody2D

## variable that stores the tile the character is on
var char_tile: Vector2i
var in_progress: bool = false

func _ready():
	# Initialize the Navigation class by calling the following function
	# NOTICE: Remember to add a custom data layer "traversable" to the tile set and set all tiles that you want to be walked on to `true`
	HexNavi.set_current_map(tile_map)
	
	#initialize tile position
	char_tile = HexNavi.global_to_cell(char.global_position)
	#snap character to the tile
	char.global_position = HexNavi.cell_to_global(char_tile)
	#mark the tile the character is at
	$TileMapLayer.set_cell_to_variant(2, char_tile)
	
	randomize_target_and_obstacles()
	
func _input(event):
	if event is InputEventMouseButton and event.is_pressed():
		if event.button_index == MOUSE_BUTTON_LEFT and !in_progress:
			var target_tile := HexNavi.global_to_cell(get_global_mouse_position())
			#check if tile exists
			if HexNavi.tile_to_id(target_tile) == -1:
				return
			#check if the tile is occupied (cannot be the final destination)
			if HexNavi.get_cell_custom_data(target_tile, "occupied"):
				return
			#get the navigation path between two tiles
			var tile_path := HexNavi.get_navi_path(char_tile, target_tile)
			
			#preview path
			for tile in tile_path:
				$TileMapLayer.set_cell_to_variant(2, tile)
				
			#move the character along the path
			in_progress = true
			var move_tween = get_tree().create_tween().set_ease(Tween.EASE_OUT).set_trans(Tween.TRANS_CUBIC)
			for tile in tile_path:
				move_tween.tween_property(char, "global_position", HexNavi.cell_to_global(tile), 0.2)
			await move_tween.finished
			char_tile = HexNavi.global_to_cell(char.global_position)
			in_progress = false
			$TileMapLayer.clear_cells()
			
			#re-apply cell highlights
			$TileMapLayer.set_cell_to_variant(2, char_tile)
			
			randomize_target_and_obstacles()
					
func randomize_target_and_obstacles():
	# set target to a random position
	var target_location := HexNavi.get_random_tile_pos()
	$Target.global_position = HexNavi.cell_to_global(target_location)
	$Target.visible = true
	#randomize obstacles
	for obstacle in $ObstacleFolder.get_children():
		obstacle.global_position = HexNavi.cell_to_global(HexNavi.get_random_tile_pos())
		obstacle.visible = true
		var obs_tile := HexNavi.global_to_cell(obstacle.global_position)
		var inpassable_region := HexNavi.get_all_neighbors_in_range(obs_tile, 1)
		for tile in inpassable_region:
			$TileMapLayer.set_cell_to_variant(1, tile)
